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
 * kernel that scales with the ball's size at that image row, fill holes, light opening. The pure-Java detector
 * does the rest (it separates touching balls), so this class is only OpenCV plumbing + the dashboard picture.
 *
 * Dashboard stream (FTC Dashboard camera view): VisionSubsystem.VIEW-like switch is {@link #VIEW}:
 *   0 = camera with every ball circled and labelled with its distance, 1 = the yellow mask, 2 = camera through the mask.
 */
public class BallVisionProcessor implements VisionProcessor, CameraStreamSource {

    /** 0 = annotated camera, 1 = mask, 2 = camera through the mask. Change it live in Dashboard (Vision Tuner shows it). */
    public static volatile int VIEW = 0;
    private static final int OVERLAY_SCALE = 2;

    public static final class Result {
        public long seq;
        public double captureS, receivedS, fps, procMs;
        public List<ClusterDetector.Detection> detections = new ArrayList<>();
    }

    private VisionParams params = new VisionParams();
    private BallCamera camera;
    private ClusterDetector detector;
    private int[][] bands;                    // {rowLo, rowHi, kernelRadius}

    private final Mat small = new Mat(), rgb = new Mat(), hsv = new Mat(), mask = new Mat(), tmp = new Mat(),
            ff = new Mat(), ffInv = new Mat(), overlay = new Mat(), maskRgba = new Mat();
    private final Mat open2 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2, 2));
    private byte[] maskBytes = new byte[0];
    private volatile Result latest;
    private long seq = 0, lastNanos = 0;
    private double fpsEma = 0;
    private volatile double[] lastHsvSample = null;      // ball-coloured pixels only (what A uses)
    private volatile double[] centerRawHsv = null;        // every pixel of the box, always set
    /** Half-size (px, in the processed image) of the centre sampling box. Tunable from the dashboard. */
    public static int CENTER_BOX_PX = 6;
    private int rawW = 640, rawH = 480;

    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    public synchronized void configure(BallCamera cam, ClusterDetector det, VisionParams v) {
        this.camera = cam;
        this.detector = det;
        this.params = v;
        // row bands with a closing radius of holeFrac * the ball radius there
        List<int[]> bl = new ArrayList<>();
        int lo = 0, cur = -1;
        for (int row = 0; row < cam.h; row++) {
            int k = Math.max(1, (int) Math.round(v.holeFrac * cam.rowRadius[row]));
            if (cur < 0) { cur = k; lo = row; }
            else if (k != cur) { bl.add(new int[]{lo, row, cur}); cur = k; lo = row; }
        }
        bl.add(new int[]{lo, cam.h, cur});
        bands = bl.toArray(new int[0][]);
        maskBytes = new byte[cam.w * cam.h];
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
    public void init(int width, int height, CameraCalibration calibration) {
        rawW = width;
        rawH = height;
    }

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
                new Scalar(VisionSubsystem.H_HIGH, VisionSubsystem.S_HIGH, VisionSubsystem.V_HIGH), mask);
        closeByBands();
        // fill the holes the closing did not: flood the background from the corner, keep what it did not reach
        Imgproc.rectangle(mask, new Point(0, 0), new Point(w - 1, h - 1), new Scalar(0), 1);
        mask.copyTo(ff);
        Mat fm = Mat.zeros(h + 2, w + 2, org.opencv.core.CvType.CV_8UC1);
        Imgproc.floodFill(ff, fm, new Point(0, 0), new Scalar(255));
        Core.bitwise_not(ff, ffInv);
        Core.bitwise_or(mask, ffInv, mask);
        fm.release();
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, open2);
        mask.get(0, 0, maskBytes);
        List<ClusterDetector.Detection> dets = detector.detect(maskBytes);
        Collections.sort(dets, (a, b) -> Double.compare(a.rangeCm(), b.rangeCm()));

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
        latest = r;
        publishBitmap(dets);
        return dets;
    }

    private void sampleCentre() {
        int cx = hsv.cols() / 2, cy = hsv.rows() / 2, n = Math.max(2, Math.min(CENTER_BOX_PX, Math.min(cx, cy) - 1));
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

    private void closeByBands() {
        int w = mask.cols(), h = mask.rows();
        for (int[] b : bands) {
            int k = b[2], lo = b[0], hi = b[1];
            int lo2 = Math.max(lo - k, 0), hi2 = Math.min(hi + k, h);
            Mat block = mask.submat(lo2, hi2, 0, w);
            Mat kern = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(2 * k + 1, 2 * k + 1));
            Imgproc.morphologyEx(block, tmp, Imgproc.MORPH_CLOSE, kern);
            tmp.submat(lo - lo2, lo - lo2 + (hi - lo), 0, w).copyTo(mask.submat(lo, hi, 0, w));
            kern.release();
            block.release();
        }
    }

    private void publishBitmap(List<ClusterDetector.Detection> dets) {
        int view = VIEW;
        int w = camera.w, h = camera.h, s = OVERLAY_SCALE;
        if (view == 1) {
            Imgproc.cvtColor(mask, maskRgba, Imgproc.COLOR_GRAY2RGBA);
            Imgproc.resize(maskRgba, overlay, new Size(w * s, h * s), 0, 0, Imgproc.INTER_NEAREST);
        } else if (view == 2) {
            Mat masked = new Mat();
            Core.bitwise_and(rgb, rgb, masked, mask);
            Imgproc.cvtColor(masked, maskRgba, Imgproc.COLOR_RGB2RGBA);
            Imgproc.resize(maskRgba, overlay, new Size(w * s, h * s), 0, 0, Imgproc.INTER_LINEAR);
            masked.release();
        } else {
            Imgproc.resize(small, overlay, new Size(w * s, h * s), 0, 0, Imgproc.INTER_LINEAR);
        }
        Scalar green = new Scalar(60, 255, 120, 255), white = new Scalar(255, 255, 255, 255);
        int i = 0;
        for (ClusterDetector.Detection d : dets) {
            i++;
            Point c = new Point(d.u * s, d.v * s);
            Imgproc.circle(overlay, c, (int) Math.round(d.radiusPx * s), green, 2);
            Imgproc.circle(overlay, c, 2, green, -1);
            String t = String.format(Locale.US, "#%d %.0fcm %+.0f", i, d.rangeCm(), d.bearingDeg());
            Imgproc.putText(overlay, t, new Point(Math.max(c.x - d.radiusPx * s, 0), Math.max(c.y - d.radiusPx * s - 4, 12)),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.42, white, 1);
        }
        // centre sampling box + the colour under it, so the HSV can be read straight off the picture
        int bx = Math.max(2, Math.min(CENTER_BOX_PX, Math.min(w, h) / 2 - 1));
        Point b0 = new Point((w / 2 - bx) * s, (h / 2 - bx) * s), b1 = new Point((w / 2 + bx) * s, (h / 2 + bx) * s);
        double[] raw = centerRawHsv;
        boolean inWin = raw != null && raw[0] >= VisionSubsystem.H_LOW && raw[0] <= VisionSubsystem.H_HIGH
                && raw[1] >= VisionSubsystem.S_LOW && raw[1] <= VisionSubsystem.S_HIGH
                && raw[2] >= VisionSubsystem.V_LOW && raw[2] <= VisionSubsystem.V_HIGH;
        Scalar boxCol = inWin ? green : new Scalar(255, 80, 80, 255);
        Imgproc.rectangle(overlay, b0, b1, boxCol, 2);
        Imgproc.line(overlay, new Point(w / 2 * s - 6, h / 2 * s), new Point(w / 2 * s + 6, h / 2 * s), boxCol, 1);
        Imgproc.line(overlay, new Point(w / 2 * s, h / 2 * s - 6), new Point(w / 2 * s, h / 2 * s + 6), boxCol, 1);
        if (raw != null) {
            Imgproc.putText(overlay, String.format(Locale.US, "HSV %.0f / %.0f / %.0f  %s", raw[0], raw[1], raw[2],
                    inWin ? "IN window" : "outside window"),
                    new Point(Math.max(b0.x - 40, 4), Math.min(b1.y + 16, overlay.rows() - 6)),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, boxCol, 1);
        }
        Imgproc.putText(overlay, String.format(Locale.US, "%.0f fps  %d balls  view %d", fpsEma, dets.size(), view),
                new Point(6, 16), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, white, 1);
        Bitmap bmp = Bitmap.createBitmap(overlay.cols(), overlay.rows(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(overlay, bmp);
        lastFrame.set(bmp);
    }

    @Override
    @SuppressWarnings("unchecked")
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx,
                            float scaleCanvasDensity, Object userContext) {
        if (!(userContext instanceof List) || camera == null) return;
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
