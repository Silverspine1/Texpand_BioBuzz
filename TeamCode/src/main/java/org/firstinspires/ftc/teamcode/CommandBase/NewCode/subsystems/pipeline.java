
package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.PipelineLogger;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class pipeline implements VisionProcessor,
        CameraStreamSource, PipelineLogger {

    // HSV SETTINGS
    public static double H_LOW = 10, H_HIGH = 30;
    public static double S_LOW = 100, S_HIGH = 255;
    public static double V_LOW = 115, V_HIGH = 255;

    // CAMERA SETTINGS
    public static double CAM_HEIGHT = 8.661417;
    public static double CAM_TILT_DOWN_DEG = 50;
    public static double CAM_H_FOV_DEG = 120;
    public static double CAM_FORWARD = 7.086614;
    public static double CAM_LEFT = 0;
    public static double CAM_YAW_DEG = 0;
    public static double TARGET_HEIGHT = 2.5;

    // BALL RADIUS IN INCHES
    public static double BALL_RADIUS_IN = 1.45;

    // DETECTION SETTINGS
    public static double FLOOR_PENALTY = 1.0;
    public static double EXPLAINED_WEIGHT = 0.8;

    public static double SCORE_MIN = 0.15;
    public static double NEW_MIN = 0.20;

    public static double DUP_FRAC = 0.8;
    public static double MAX_RANGE_IN = 120;

    public static int MAX_BALLS = 10;

    // 0 = camera + overlay
    // 1 = mask
    // 2 = thresholded camera
    public static int VIEW = 1;

    // OPENCV OBJECTS
    private final Mat rgb = new Mat();
    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();
    private final Mat tmp = new Mat();

    private final Mat kernel = Imgproc.getStructuringElement(
            Imgproc.MORPH_RECT,
            new Size(3, 3)
    );

    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(
                    Bitmap.createBitmap(
                            1, 1, Bitmap.Config.RGB_565
                    )
            );

    // OUTPUTS
    private volatile Rect bestRect = null;
    private volatile Point center = null;

    private volatile double[] targetHsv = null;
    private volatile double[] centerHsv = null;
    private volatile double[] robotXY = null;

    private volatile List<Detection> detections =
            Collections.emptyList();

    private volatile List<MatOfPoint> contours =
            new ArrayList<>();

    // CANDIDATE CLASS
    private static class Candidate {

        int u, v;
        int k;

        int area;
        int M;
        int Ex = 0;

        double score;

        double gx, gy;

        boolean alive = true;

        Candidate(int u, int v, int k) {
            this.u = u;
            this.v = v;
            this.k = k;
        }
    }

    // DETECTION CLASS
    public static class Detection {

        public final double u, v;
        public final double radiusPx;
        public final double score;
        public final double coverage;

        // Robot coordinates in inches
        public final double xr, yr;

        Detection(
                double u,
                double v,
                double radiusPx,
                double score,
                double coverage,
                double xr,
                double yr
        ) {
            this.u = u;
            this.v = v;
            this.radiusPx = radiusPx;
            this.score = score;
            this.coverage = coverage;
            this.xr = xr;
            this.yr = yr;
        }
    }

    // INITIALIZATION
    @Override
    public void init(
            int width,
            int height,
            CameraCalibration calibration
    ) {
        lastFrame.set(
                Bitmap.createBitmap(
                        width,
                        height,
                        Bitmap.Config.RGB_565
                )
        );
    }

    // MAIN FRAME PROCESSING
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        int w = frame.cols();
        int h = frame.rows();
        
        Imgproc.cvtColor(frame, rgb, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV);

        Core.inRange(
                hsv,
                new Scalar(H_LOW, S_LOW, V_LOW),
                new Scalar(H_HIGH, S_HIGH, V_HIGH),
                mask
        );

        Imgproc.morphologyEx(
                mask, mask, Imgproc.MORPH_OPEN, kernel
        );

        Imgproc.morphologyEx(
                mask, mask, Imgproc.MORPH_CLOSE, kernel
        );

        centerHsv = hsv.get(h / 2, w / 2);

        byte[] maskData = new byte[w * h];
        mask.get(0, 0, maskData);

        List<Detection> foundBalls =
                detectBalls(maskData, w, h);

        detections = Collections.unmodifiableList(
                new ArrayList<>(foundBalls)
        );

        Detection first = foundBalls.isEmpty()
                ? null : foundBalls.get(0);

        Point ctr = null;
        double[] tHsv = null;
        double[] xy = null;
        Rect rect = null;

        if (first != null) {

            ctr = new Point(first.u, first.v);

            tHsv = hsv.get(
                    (int) Math.round(first.v),
                    (int) Math.round(first.u)
            );

            xy = new double[]{first.xr, first.yr};

            int r = (int) Math.round(first.radiusPx);

            int left = Math.max(0, (int) first.u - r);
            int top = Math.max(0, (int) first.v - r);
            int right = Math.min(w, (int) first.u + r + 1);
            int bottom = Math.min(h, (int) first.v + r + 1);

            rect = new Rect(
                    left, top,
                    right - left,
                    bottom - top
            );
        }

        center = ctr;
        bestRect = rect;
        targetHsv = tHsv;
        robotXY = xy;

        if (VIEW == 1) {

            Imgproc.cvtColor(
                    mask, frame, Imgproc.COLOR_GRAY2RGBA
            );

        } else if (VIEW == 2) {

            rgb.copyTo(tmp);
            tmp.setTo(new Scalar(0, 0, 0));
            rgb.copyTo(tmp, mask);

            Imgproc.cvtColor(
                    tmp, frame, Imgproc.COLOR_RGB2RGBA
            );
        }

        Scalar white = new Scalar(255, 255, 255, 255);
        Scalar green = new Scalar(0, 255, 0, 255);
        Scalar red = new Scalar(255, 0, 0, 255);

        Imgproc.drawMarker(
                frame,
                new Point(w / 2, h / 2),
                red,
                Imgproc.MARKER_CROSS,
                20,
                2
        );

        putLabel(
                frame,
                "Balls: " + foundBalls.size(),
                18,
                white
        );

        putLabel(
                frame,
                String.format(
                        "H %.0f-%.0f S %.0f-%.0f V %.0f-%.0f",
                        H_LOW, H_HIGH,
                        S_LOW, S_HIGH,
                        V_LOW, V_HIGH
                ),
                38,
                white
        );

        for (int i = 0; i < foundBalls.size(); i++) {

            Detection d = foundBalls.get(i);

            Point p = new Point(d.u, d.v);

            Imgproc.circle(
                    frame,
                    p,
                    (int) Math.round(d.radiusPx),
                    green,
                    2
            );

            Imgproc.drawMarker(
                    frame,
                    p,
                    red,
                    Imgproc.MARKER_CROSS,
                    12,
                    2
            );

            Imgproc.putText(
                    frame,
                    String.format("#%d %.2f", i + 1, d.score),
                    new Point(d.u + 5, d.v - 10),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    green,
                    2
            );

            Imgproc.putText(
                    frame,
                    String.format("X %.1f Y %.1f", d.xr, d.yr),
                    new Point(d.u + 5, d.v + 15),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    white,
                    1
            );
        }

        Bitmap bmp = Bitmap.createBitmap(
                w, h, Bitmap.Config.RGB_565
        );

        Utils.matToBitmap(frame, bmp);

        Bitmap old = lastFrame.getAndSet(bmp);

        if (old != null && !old.isRecycled()) {
            old.recycle();
        }

        //frame = mask;

        return foundBalls;
    }

    // BUILD RADIUS MAP
    private int[] buildKMap(int w, int h) {

        int[] kmap = new int[w * h];

        double f = (w / 2.0) /
                Math.tan(
                        Math.toRadians(CAM_H_FOV_DEG) / 2.0
                );

        double tilt = Math.toRadians(CAM_TILT_DOWN_DEG);

        for (int v = 0; v < h; v++) {

            for (int u = 0; u < w; u++) {

                int index = v * w + u;

                double xc = (u - w / 2.0) / f;
                double yc = (v - h / 2.0) / f;

                double denom =
                        Math.sin(tilt) +
                                yc * Math.cos(tilt);

                if (denom <= 1e-6) {
                    kmap[index] = 0;
                    continue;
                }

                double t =
                        (CAM_HEIGHT - TARGET_HEIGHT) / denom;

                double distance =
                        t * Math.sqrt(1 + xc * xc + yc * yc);

                double radiusPx =
                        f * BALL_RADIUS_IN / distance;

                kmap[index] = (int) Math.round(radiusPx);
            }
        }

        return kmap;
    }

    // BUILD PREFIX SUMS
    private int[][] buildPrefix(
            byte[] data,
            int w,
            int h
    ) {

        int[][] pref = new int[h][w + 1];

        for (int y = 0; y < h; y++) {

            for (int x = 0; x < w; x++) {

                pref[y][x + 1] =
                        pref[y][x] +
                                ((data[y * w + x] & 0xFF) != 0 ? 1 : 0);
            }
        }

        return pref;
    }

    // COUNT PIXELS IN DISC
    private int disc(
            int[][] pref,
            int u,
            int v,
            int k,
            int w,
            int h
    ) {

        int count = 0;

        for (int dy = -k; dy <= k; dy++) {

            int y = v + dy;

            if (y < 0 || y >= h) continue;

            int hw = (int) Math.floor(
                    Math.sqrt(k * k + 0.25 - dy * dy)
            );

            int left = Math.max(0, u - hw);
            int right = Math.min(w - 1, u + hw);

            count += pref[y][right + 1] - pref[y][left];
        }

        return count;
    }

    // DISC AREA LOOKUP
    private int[] buildAreaLookup(int maxK) {

        int[] areas = new int[maxK + 1];

        for (int k = 0; k <= maxK; k++) {

            int area = 0;

            for (int dy = -k; dy <= k; dy++) {

                int hw = (int) Math.floor(
                        Math.sqrt(k * k + 0.25 - dy * dy)
                );

                area += 2 * hw + 1;
            }

            areas[k] = area;
        }

        return areas;
    }

    // BUILD CANDIDATES
    private List<Candidate> buildCandidates(
            byte[] maskData,
            int[] kmap,
            int[] areaLookup,
            int[][] prefM,
            int w,
            int h
    ) {

        List<Candidate> candidates = new ArrayList<>();

        int maxK = Math.min(w, h) / 2;

        for (int v = 1; v < h - 1; v++) {

            for (int u = 1; u < w - 1; u++) {

                int i = v * w + u;

                if ((maskData[i] & 0xFF) == 0) continue;

                if ((maskData[i - 1] & 0xFF) == 0) continue;
                if ((maskData[i + 1] & 0xFF) == 0) continue;
                if ((maskData[i - w] & 0xFF) == 0) continue;
                if ((maskData[i + w] & 0xFF) == 0) continue;

                int k = kmap[i];

                if (k < 2 || k > maxK) continue;

                if (u - k < 0 || u + k >= w) continue;
                if (v - k < 0 || v + k >= h) continue;

                Candidate c = new Candidate(u, v, k);

                c.area = areaLookup[k];

                c.M = disc(
                        prefM, u, v, k, w, h
                );

                c.score = (
                        c.M -
                                FLOOR_PENALTY * (c.area - c.M)
                ) / (double) c.area;

                double[] xy = pixelToRobot(
                        u + 0.5, v + 0.5, w, h
                );

                if (xy == null) continue;

                c.gx = xy[0];
                c.gy = xy[1];

                candidates.add(c);
            }
        }

        return candidates;
    }

    // REFINE CENTRE
    private Point refineCentre(
            List<Candidate> candidates,
            Candidate best
    ) {

        int wn = Math.max(
                1,
                (int) Math.round(0.18 * best.k)
        );

        double sumU = 0;
        double sumV = 0;
        double sumW = 0;

        for (Candidate c : candidates) {

            if (!c.alive) continue;

            double dx = c.u - best.u;
            double dy = c.v - best.v;

            if (dx * dx + dy * dy > wn * wn) continue;

            double wt = Math.max(
                    c.score - (best.score - 0.06),
                    0
            );

            if (wt <= 0) continue;

            sumU += wt * (c.u + 0.5);
            sumV += wt * (c.v + 0.5);
            sumW += wt;
        }

        if (sumW == 0) {
            return new Point(best.u + 0.5, best.v + 0.5);
        }

        return new Point(sumU / sumW, sumV / sumW);
    }

    // DUPLICATE CHECK
    private boolean isDuplicate(
            List<Detection> results,
            double x,
            double y
    ) {

        double minDistance =
                DUP_FRAC * 2.0 * BALL_RADIUS_IN;

        for (Detection d : results) {

            double dx = d.xr - x;
            double dy = d.yr - y;

            if (Math.hypot(dx, dy) < minDistance) {
                return true;
            }
        }

        return false;
    }

    // MARK EXPLAINED PIXELS
    private void markExplained(
            byte[] maskData,
            byte[] explained,
            int u,
            int v,
            int k,
            int w,
            int h
    ) {

        for (int dy = -k; dy <= k; dy++) {

            for (int dx = -k; dx <= k; dx++) {

                if (dx * dx + dy * dy > k * k) continue;

                int x = u + dx;
                int y = v + dy;

                if (x < 0 || x >= w ||
                        y < 0 || y >= h) continue;

                int index = y * w + x;

                if ((maskData[index] & 0xFF) != 0) {
                    explained[index] = 1;
                }
            }
        }
    }

    // GREEDY MULTI-BALL DETECTOR
    private List<Detection> detectBalls(
            byte[] maskData,
            int w,
            int h
    ) {

        List<Detection> results = new ArrayList<>();

        int[] kmap = buildKMap(w, h);

        int[][] prefM = buildPrefix(maskData, w, h);

        int[] areaLookup =
                buildAreaLookup(Math.min(w, h) / 2);

        List<Candidate> candidates = buildCandidates(
                maskData,
                kmap,
                areaLookup,
                prefM,
                w,
                h
        );

        byte[] explained = new byte[w * h];

        for (int iteration = 0;
             iteration < MAX_BALLS;
             iteration++) {

            Candidate best = null;

            // Find highest scoring candidate
            for (Candidate c : candidates) {

                if (!c.alive) continue;

                if (best == null || c.score > best.score) {
                    best = c;
                }
            }

            if (best == null || best.score < SCORE_MIN) {
                break;
            }

            // Refine centre
            Point refined = refineCentre(candidates, best);

            // Back-project
            double[] xy = pixelToRobot(
                    refined.x, refined.y, w, h
            );

            if (xy == null) {
                best.alive = false;
                continue;
            }

            double xr = xy[0];
            double yr = xy[1];

            // Remove duplicate detections
            if (isDuplicate(results, xr, yr)) {
                best.alive = false;
                continue;
            }

            // Range check
            if (Math.hypot(xr, yr) > MAX_RANGE_IN) {
                best.alive = false;
                continue;
            }

            // Fresh coverage
            double freshCoverage =
                    (best.M - best.Ex) / (double) best.area;

            if (freshCoverage < NEW_MIN) {
                best.alive = false;
                continue;
            }

            // Accept detection
            results.add(
                    new Detection(
                            refined.x,
                            refined.y,
                            best.k,
                            best.score,
                            best.M / (double) best.area,
                            xr,
                            yr
                    )
            );

            // Mark explained pixels
            markExplained(
                    maskData,
                    explained,
                    (int) Math.round(refined.x),
                    (int) Math.round(refined.y),
                    best.k,
                    w,
                    h
            );

            // Rebuild explained prefix sums
            int[][] prefEx = buildPrefix(
                    explained, w, h
            );

            // Rescore candidates
            for (Candidate c : candidates) {

                if (!c.alive) continue;

                double dx = c.gx - xr;
                double dy = c.gy - yr;

                double distance = Math.hypot(dx, dy);

                if (distance <
                        DUP_FRAC * 2.0 * BALL_RADIUS_IN) {

                    c.alive = false;
                    continue;
                }

                c.Ex = disc(
                        prefEx,
                        c.u,
                        c.v,
                        c.k,
                        w,
                        h
                );

                c.score = (
                        c.M
                                - EXPLAINED_WEIGHT * c.Ex
                                - FLOOR_PENALTY * (c.area - c.M)
                ) / (double) c.area;
            }
        }

        return results;
    }

    // PIXEL TO ROBOT COORDINATES
    private static double[] pixelToRobot(
            double u,
            double v,
            int w,
            int h
    ) {

        double f = (w / 2.0) /
                Math.tan(
                        Math.toRadians(CAM_H_FOV_DEG) / 2.0
                );

        double xc = (u - w / 2.0) / f;
        double yc = (v - h / 2.0) / f;

        double tilt = Math.toRadians(CAM_TILT_DOWN_DEG);

        double denom =
                Math.sin(tilt) +
                        yc * Math.cos(tilt);

        if (denom <= 1e-6) return null;

        double t =
                (CAM_HEIGHT - TARGET_HEIGHT) / denom;

        double fwd =
                t * (
                        Math.cos(tilt) -
                                yc * Math.sin(tilt)
                );

        double left = -t * xc;

        double yaw = Math.toRadians(CAM_YAW_DEG);

        double rx =
                fwd * Math.cos(yaw)
                        - left * Math.sin(yaw)
                        + CAM_FORWARD;

        double ry =
                fwd * Math.sin(yaw)
                        + left * Math.cos(yaw)
                        + CAM_LEFT;

        return new double[]{rx, ry};
    }

    // DRAW LABEL
    private static void putLabel(
            Mat frame,
            String text,
            int y,
            Scalar color
    ) {

        Imgproc.putText(
                frame,
                text,
                new Point(6, y),
                Imgproc.FONT_HERSHEY_SIMPLEX,
                0.55,
                new Scalar(0, 0, 0, 255),
                3
        );

        Imgproc.putText(
                frame,
                text,
                new Point(6, y),
                Imgproc.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                1
        );
    }

    // FTC CAMERA STREAM
    @Override
    public void onDrawFrame(
            Canvas canvas,
            int onscreenWidth,
            int onscreenHeight,
            float scaleBmpPxToCanvasPx,
            float scaleCanvasDensity,
            Object userContext
    ) {
    }

    @Override
    public void getFrameBitmap(
            Continuation<? extends Consumer<Bitmap>> continuation
    ) {

        continuation.dispatch(
                bitmapConsumer ->
                        bitmapConsumer.accept(lastFrame.get())
        );
    }

    // GETTERS
    public Point getCenter() {
        return center;
    }

    public Rect getRect() {
        return bestRect;
    }

    public List<MatOfPoint> getContours() {
        return contours;
    }

    public double[] getTargetHsv() {
        return targetHsv == null ? null : targetHsv.clone();
    }

    public double[] getCenterHsv() {
        return centerHsv == null ? null : centerHsv.clone();
    }

    public double[] getRobotXY() {
        return robotXY == null ? null : robotXY.clone();
    }

    public List<Detection> getDetections() {
        return new ArrayList<>(detections);
    }

    public int getDetectionCount() {
        return detections.size();
    }
}
