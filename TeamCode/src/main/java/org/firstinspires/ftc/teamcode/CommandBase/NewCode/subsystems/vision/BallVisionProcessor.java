package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Camera frame -> yellow mask -> {@link ClusterDetector}. Runs on the camera thread.
 *
 * Mask: resize to the work size, HSV threshold (VisionSubsystem H/S/V), close the ball's perforations with a
 * kernel that scales with the ball's size AT THAT PIXEL (per-pixel, the lens makes size vary across the row),
 * fill holes, light opening. The pure-Java detector does the rest (it separates touching balls).
 *
 * Dashboard camera stream AND the Driver Station camera preview, chosen by VisionSubsystem.VIEW (a Dashboard
 * number; X on gamepad1 in the Tuner toggles camera / mask):
 *   0 = camera + circled balls    1 = the yellow mask    2 = camera through the mask    3 = camera | mask side by side
 * The mask panel also shows the single-ball size check: the CYAN circle is the ball blob's measured radius, the RED
 * circle is the radius the camera model expects there. Red much smaller than cyan => HFOV too big / lens mix wrong.
 */
public class BallVisionProcessor implements VisionProcessor, CameraStreamSource {

    private static final int OVERLAY_SCALE = 2;

    public static final class Blob {
        public double u, v, areaPx, measuredR, expectedR;
        public double ratio() { return expectedR > 0 ? measuredR / expectedR : 0; }
    }

    public static final class Result {
        public long seq;
        public double captureS, receivedS, fps, procMs;
        public List<ClusterDetector.Detection> detections = new ArrayList<>();
        /** Largest mask blob (the ball when ONE ball is in view), or null. */
        public Blob blob;
        /** How many separate mask blobs (>30 px) there are - 1 means exactly one ball (or one touching group) in view. */
        public int blobCount;
    }

    private VisionParams params = new VisionParams();
    private BallCamera camera;
    private ClusterDetector detector;
    private final List<Mat> levelMasks = new ArrayList<>();      // 255 where the closing kernel radius == level
    private final List<Mat> levelKernels = new ArrayList<>();

    private final Mat small = new Mat(), rgb = new Mat(), hsv = new Mat(), mask = new Mat(), raw = new Mat(),
            closedTmp = new Mat(), ff = new Mat(), ffInv = new Mat(), camPanel = new Mat(), maskPanel = new Mat(),
            maskRgba = new Mat(), labels = new Mat(), stats = new Mat(), centroids = new Mat(), out = new Mat();
    private final Mat open2 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2, 2));
    private byte[] maskBytes = new byte[0];
    private volatile Result latest;
    private long seq = 0, lastNanos = 0;
    private double fpsEma = 0;
    private volatile double[] lastHsvSample = null;      // ball-coloured pixels only (what A uses)
    private volatile double[] centerRawHsv = null;        // every pixel of the box, always set

    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    public synchronized void configure(BallCamera cam, ClusterDetector det, VisionParams v) {
        this.camera = cam;
        this.detector = det;
        this.params = v;
        for (Mat m : levelMasks) m.release();
        for (Mat m : levelKernels) m.release();
        levelMasks.clear();
        levelKernels.clear();
        // closing kernel radius per pixel = holeFrac * the ball radius expected at that pixel
        int w = cam.w, h = cam.h;
        int[] kh = new int[w * h];
        java.util.TreeSet<Integer> levels = new java.util.TreeSet<>();
        for (int i = 0; i < w * h; i++) {
            kh[i] = cam.pixelRadius[i] > 0 ? Math.max(1, (int) Math.round(v.holeFrac * cam.pixelRadius[i])) : 1;
            levels.add(kh[i]);
        }
        for (int k : levels) {
            byte[] data = new byte[w * h];
            for (int i = 0; i < w * h; i++) if (kh[i] == k) data[i] = (byte) 255;
            Mat lm = new Mat(h, w, CvType.CV_8UC1);
            lm.put(0, 0, data);
            levelMasks.add(lm);
            levelKernels.add(Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(2 * k + 1, 2 * k + 1)));
        }
        maskBytes = new byte[w * h];
    }

    public synchronized void setParams(VisionParams v) {
        this.params = v;
    }

    public Result latest() {
        return latest;
    }

    /** Median HSV (excluding dark holes) of a small box at the image centre, or null. Used by the Vision Tuner. */
    public double[] sampleCenterHsv() {
        return lastHsvSample;
    }

    /** Median HSV of EVERY pixel in the centre box (floor, holes and all), or null before the first frame. */
    public double[] centerRawHsv() {
        return centerRawHsv;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) { }

    @Override
    public synchronized Object processFrame(Mat frame, long captureTimeNanos) {
        if (camera == null || detector == null) return null;
        long t0 = System.nanoTime();
        int w = camera.w, h = camera.h;
        Imgproc.resize(frame, small, new Size(w, h), 0, 0, Imgproc.INTER_AREA);
        Imgproc.cvtColor(small, rgb, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV);
        sampleCentre();
        Core.inRange(hsv, new Scalar(VisionSubsystem.H_LOW, VisionSubsystem.S_LOW, VisionSubsystem.V_LOW),
                new Scalar(VisionSubsystem.H_HIGH, VisionSubsystem.S_HIGH, VisionSubsystem.V_HIGH), raw);
        closePerPixel();
        // fill the holes the closing did not: flood the background from the corner, keep what it did not reach
        Imgproc.rectangle(mask, new Point(0, 0), new Point(w - 1, h - 1), new Scalar(0), 1);
        mask.copyTo(ff);
        Mat fm = Mat.zeros(h + 2, w + 2, CvType.CV_8UC1);
        Imgproc.floodFill(ff, fm, new Point(0, 0), new Scalar(255));
        Core.bitwise_not(ff, ffInv);
        Core.bitwise_or(mask, ffInv, mask);
        fm.release();
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, open2);
        mask.get(0, 0, maskBytes);
        List<ClusterDetector.Detection> dets = detector.detect(maskBytes);
        Collections.sort(dets, (a, b) -> Double.compare(a.rangeCm(), b.rangeCm()));
        Blob blob = largestBlob();

        long now = System.nanoTime();
        double dtS = lastNanos == 0 ? 0 : (now - lastNanos) * 1e-9;
        lastNanos = now;
        if (dtS > 0) fpsEma = fpsEma == 0 ? 1.0 / dtS : 0.9 * fpsEma + 0.1 / dtS;
        Result r = new Result();
        r.seq = ++seq;
        r.captureS = captureTimeNanos * 1e-9;
        r.receivedS = now * 1e-9;
        r.fps = fpsEma;
        r.procMs = (now - t0) * 1e-6;
        r.detections = dets;
        r.blob = blob;
        r.blobCount = lastBlobCount;
        latest = r;
        publishBitmap(dets, blob);
        return dets;
    }

    private void sampleCentre() {
        int cx = hsv.cols() / 2, cy = hsv.rows() / 2;
        int n = Math.max(2, Math.min(VisionSubsystem.CENTER_BOX_PX, Math.min(cx, cy) - 1));
        Rect box = new Rect(cx - n, cy - n, 2 * n, 2 * n);
        Mat sub = hsv.submat(box);
        byte[] data = new byte[sub.cols() * sub.rows() * 3];
        sub.get(0, 0, data);
        sub.release();
        List<Integer> ah = new ArrayList<>(), as = new ArrayList<>(), av = new ArrayList<>();
        for (int i = 0; i + 2 < data.length; i += 3) {
            ah.add(data[i] & 0xFF); as.add(data[i + 1] & 0xFF); av.add(data[i + 2] & 0xFF);
        }
        Collections.sort(ah); Collections.sort(as); Collections.sort(av);
        int am = ah.size() / 2;
        centerRawHsv = ah.isEmpty() ? null : new double[]{ah.get(am), as.get(am), av.get(am)};
        List<Integer> hs = new ArrayList<>(), ss = new ArrayList<>(), vs = new ArrayList<>();
        for (int i = 0; i + 2 < data.length; i += 3) {
            int hh = data[i] & 0xFF, s = data[i + 1] & 0xFF, vv = data[i + 2] & 0xFF;
            if (s > 60 && vv > 60) { hs.add(hh); ss.add(s); vs.add(vv); }     // skip the dark holes / floor
        }
        if (hs.size() < 8) { lastHsvSample = null; return; }
        Collections.sort(hs); Collections.sort(ss); Collections.sort(vs);
        int m = hs.size() / 2;
        lastHsvSample = new double[]{hs.get(m), ss.get(m), vs.get(m)};
    }

    /** Close the ball perforations with a kernel sized for the ball AT EACH PIXEL (one closing per size level). */
    private void closePerPixel() {
        mask.create(raw.rows(), raw.cols(), CvType.CV_8UC1);
        mask.setTo(new Scalar(0));
        for (int i = 0; i < levelMasks.size(); i++) {
            Imgproc.morphologyEx(raw, closedTmp, Imgproc.MORPH_CLOSE, levelKernels.get(i));
            closedTmp.copyTo(mask, levelMasks.get(i));
        }
    }

    private int lastBlobCount = 0;

    private Blob largestBlob() {
        int n = Imgproc.connectedComponentsWithStats(mask, labels, stats, centroids, 8, CvType.CV_32S);
        int best = -1, bestArea = 30, count = 0;
        for (int i = 1; i < n; i++) {
            int a = (int) stats.get(i, Imgproc.CC_STAT_AREA)[0];
            if (a > 30) count++;
            if (a > bestArea) { bestArea = a; best = i; }
        }
        lastBlobCount = count;
        if (best < 0) return null;
        Blob b = new Blob();
        b.u = centroids.get(best, 0)[0];
        b.v = centroids.get(best, 1)[0];
        b.areaPx = bestArea;
        b.measuredR = Math.sqrt(bestArea / Math.PI);
        int iu = Math.max(0, Math.min(camera.w - 1, (int) b.u)), iv = Math.max(0, Math.min(camera.h - 1, (int) b.v));
        b.expectedR = camera.pixelRadius[iv * camera.w + iu];
        return b;
    }

    private void publishBitmap(List<ClusterDetector.Detection> dets, Blob blob) {
        int view = VisionSubsystem.VIEW;
        int w = camera.w, h = camera.h, s = OVERLAY_SCALE;
        Scalar green = new Scalar(60, 255, 120, 255), white = new Scalar(255, 255, 255, 255),
                red = new Scalar(255, 70, 70, 255), cyan = new Scalar(0, 220, 255, 255);
        boolean wantCam = view != 1, wantMask = view == 1 || view == 3;
        if (wantCam) {
            if (view == 2) {
                Mat masked = new Mat();
                Core.bitwise_and(rgb, rgb, masked, mask);
                Imgproc.cvtColor(masked, maskRgba, Imgproc.COLOR_RGB2RGBA);
                Imgproc.resize(maskRgba, camPanel, new Size(w * s, h * s), 0, 0, Imgproc.INTER_LINEAR);
                masked.release();
            } else {
                Imgproc.resize(small, camPanel, new Size(w * s, h * s), 0, 0, Imgproc.INTER_LINEAR);
            }
            int i = 0;
            for (ClusterDetector.Detection d : dets) {
                i++;
                Point c = new Point(d.u * s, d.v * s);
                Imgproc.circle(camPanel, c, (int) Math.round(d.radiusPx * s), green, 2);
                Imgproc.circle(camPanel, c, 2, green, -1);
                String t = String.format(Locale.US, "#%d %.0fcm %+.0f", i, d.rangeCm(), d.bearingDeg());
                Imgproc.putText(camPanel, t, new Point(Math.max(c.x - d.radiusPx * s, 0), Math.max(c.y - d.radiusPx * s - 4, 12)),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.42, white, 1);
            }
            if (VisionSubsystem.SHOW_EXPECTED_SIZE) {
                // reference discs: put a real ball on each and compare - the circle is the size the model expects there
                double[] fu = {0.15, 0.5, 0.85}, fv = {0.35, 0.6, 0.85};
                for (double a : fu) for (double b : fv) {
                    int iu = (int) (a * w), iv = (int) (b * h);
                    double rr = camera.pixelRadius[iv * w + iu];
                    if (rr > 1) Imgproc.circle(camPanel, new Point(iu * s, iv * s), (int) Math.round(rr * s), red, 1);
                }
            }
            drawCentreBox(camPanel, s, w, h, green, white);
            Imgproc.putText(camPanel, String.format(Locale.US, "%.0f fps  %d balls  view %d", fpsEma, dets.size(), view),
                    new Point(6, 16), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, white, 1);
        }
        if (wantMask) {
            Imgproc.cvtColor(mask, maskRgba, Imgproc.COLOR_GRAY2RGBA);
            Imgproc.resize(maskRgba, maskPanel, new Size(w * s, h * s), 0, 0, Imgproc.INTER_NEAREST);
            if (blob != null) {
                Point c = new Point(blob.u * s, blob.v * s);
                Imgproc.circle(maskPanel, c, (int) Math.round(blob.measuredR * s), cyan, 2);
                Imgproc.circle(maskPanel, c, (int) Math.round(blob.expectedR * s), red, 2);
                Imgproc.putText(maskPanel, String.format(Locale.US, "blob r=%.1f  model expects %.1f  ratio %.2f",
                        blob.measuredR, blob.expectedR, blob.ratio()), new Point(6, 16), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, cyan, 1);
            } else {
                Imgproc.putText(maskPanel, "mask (no blob)", new Point(6, 16), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, cyan, 1);
            }
            // the ball circles and the centre sampling box with its live HSV, so HSV can be tuned on the mask itself
            for (ClusterDetector.Detection d : dets) {
                Imgproc.circle(maskPanel, new Point(d.u * s, d.v * s), (int) Math.round(d.radiusPx * s), green, 2);
            }
            drawCentreBox(maskPanel, s, w, h, green, white);
            Imgproc.putText(maskPanel, String.format(Locale.US, "H %.0f-%.0f S %.0f-%.0f V %.0f-%.0f", VisionSubsystem.H_LOW,
                    VisionSubsystem.H_HIGH, VisionSubsystem.S_LOW, VisionSubsystem.S_HIGH, VisionSubsystem.V_LOW, VisionSubsystem.V_HIGH),
                    new Point(6, h * s - 8), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, white, 1);
        }
        Mat show;
        if (view == 1) show = maskPanel;
        else if (view == 3) {
            out.create(h * s, w * s * 2, camPanel.type());
            camPanel.copyTo(out.submat(0, h * s, 0, w * s));
            maskPanel.copyTo(out.submat(0, h * s, w * s, w * s * 2));
            show = out;
        } else show = camPanel;
        Bitmap bmp = Bitmap.createBitmap(show.cols(), show.rows(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(show, bmp);
        lastFrame.set(bmp);
    }

    /** Centre sampling box + the colour under it, so the HSV can be read straight off the picture. */
    private void drawCentreBox(Mat panel, int s, int w, int h, Scalar green, Scalar white) {
        int bx = Math.max(2, Math.min(VisionSubsystem.CENTER_BOX_PX, Math.min(w, h) / 2 - 1));
        Point b0 = new Point((w / 2 - bx) * s, (h / 2 - bx) * s), b1 = new Point((w / 2 + bx) * s, (h / 2 + bx) * s);
        double[] rawHsv = centerRawHsv;
        boolean inWin = rawHsv != null && rawHsv[0] >= VisionSubsystem.H_LOW && rawHsv[0] <= VisionSubsystem.H_HIGH
                && rawHsv[1] >= VisionSubsystem.S_LOW && rawHsv[1] <= VisionSubsystem.S_HIGH
                && rawHsv[2] >= VisionSubsystem.V_LOW && rawHsv[2] <= VisionSubsystem.V_HIGH;
        Scalar col = inWin ? green : new Scalar(255, 80, 80, 255);
        Imgproc.rectangle(panel, b0, b1, col, 2);
        Imgproc.line(panel, new Point(w / 2 * s - 6, h / 2 * s), new Point(w / 2 * s + 6, h / 2 * s), col, 1);
        Imgproc.line(panel, new Point(w / 2 * s, h / 2 * s - 6), new Point(w / 2 * s, h / 2 * s + 6), col, 1);
        if (rawHsv != null) {
            Imgproc.putText(panel, String.format(Locale.US, "HSV %.0f / %.0f / %.0f  %s", rawHsv[0], rawHsv[1], rawHsv[2],
                    inWin ? "IN window" : "outside window"),
                    new Point(Math.max(b0.x - 40, 4), Math.min(b1.y + 16, panel.rows() - 6)),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, col, 1);
        }
    }

    @Override
    @SuppressWarnings("unchecked")
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx,
                            float scaleCanvasDensity, Object userContext) {
        if (!(userContext instanceof List) || camera == null) return;
        Bitmap shown = lastFrame.get();
        if (VisionSubsystem.VIEW != 0 && shown != null) {
            // Driver Station preview: show the selected view (mask etc.), overlays included
            canvas.drawBitmap(shown, null, new android.graphics.Rect(0, 0, onscreenWidth, onscreenHeight), null);
            return;
        }
        Paint p = new Paint();
        p.setColor(Color.GREEN);
        p.setStyle(Paint.Style.STROKE);
        p.setStrokeWidth(3);
        float sx = (float) onscreenWidth / camera.w, sy = (float) onscreenHeight / camera.h;
        for (ClusterDetector.Detection d : (List<ClusterDetector.Detection>) userContext) {
            canvas.drawCircle((float) d.u * sx, (float) d.v * sy, (float) d.radiusPx * sx, p);
        }
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
}
