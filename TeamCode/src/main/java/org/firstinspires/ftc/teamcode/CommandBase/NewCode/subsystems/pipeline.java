package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
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
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class pipeline implements VisionProcessor, CameraStreamSource {

    // HSV thresholds, OpenCV ranges: H 0-179, S/V 0-255. Edit live in FTC Dashboard.
    public static double H_LOW = 40, H_HIGH = 85;
    public static double S_LOW = 80, S_HIGH = 255;
    public static double V_LOW = 60, V_HIGH = 255;
    public static double MIN_AREA = 300;

    // 0 = camera + overlay, 1 = mask only, 2 = camera masked by threshold
    public static int VIEW = 0;

    // Camera placement relative to robot center (inches / degrees). Robot frame: +x forward, +y left.
    public static double CAM_HEIGHT = 10;
    public static double CAM_TILT_DOWN_DEG = 30;
    public static double CAM_H_FOV_DEG = 60;
    public static double CAM_FORWARD = 0;
    public static double CAM_LEFT = 0;
    public static double CAM_YAW_DEG = 0;
    // Height of the target's center above the floor
    public static double TARGET_HEIGHT = 2.5;

    private final Mat rgb = new Mat();
    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();
    private final Mat tmp = new Mat();
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));

    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    private volatile Rect bestRect = null;
    private volatile Point center = null;
    private volatile double[] targetHsv = null;
    private volatile double[] centerHsv = null;
    private volatile double[] robotXY = null;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // VisionPortal frames are RGBA
        Imgproc.cvtColor(frame, rgb, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, new Scalar(H_LOW, S_LOW, V_LOW), new Scalar(H_HIGH, S_HIGH, V_HIGH), mask);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        // findContours may modify its input, so give it a copy
        mask.copyTo(tmp);
        Imgproc.findContours(tmp, contours, hierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        hierarchy.release();

        Rect best = null;
        double bestArea = MIN_AREA;
        for (MatOfPoint c : contours) {
            double area = Imgproc.contourArea(c);
            if (area > bestArea) {
                bestArea = area;
                best = Imgproc.boundingRect(c);
            }
            c.release();
        }

        int w = frame.cols(), h = frame.rows();
        int cx = w / 2, cy = h / 2;
        centerHsv = hsv.get(cy, cx);

        Point ctr = null;
        double[] tHsv = null;
        double[] xy = null;
        if (best != null) {
            ctr = new Point(best.x + best.width / 2.0, best.y + best.height / 2.0);
            tHsv = hsv.get((int) ctr.y, (int) ctr.x);
            xy = pixelToRobot(ctr.x, ctr.y, w, h);
        }
        bestRect = best;
        center = ctr;
        targetHsv = tHsv;
        robotXY = xy;

        if (VIEW == 1) {
            Imgproc.cvtColor(mask, frame, Imgproc.COLOR_GRAY2RGBA);
        } else if (VIEW == 2) {
            frame.copyTo(tmp, mask);
            tmp.copyTo(frame);
        }

        Scalar white = new Scalar(255, 255, 255, 255);
        Scalar green = new Scalar(0, 255, 0, 255);
        Scalar red = new Scalar(255, 0, 0, 255);
        Imgproc.drawMarker(frame, new Point(cx, cy), red, Imgproc.MARKER_CROSS, 20, 2);
        putLabel(frame, String.format("Center HSV %.0f %.0f %.0f",
                centerHsv[0], centerHsv[1], centerHsv[2]), 18, white);
        putLabel(frame, String.format("Thresh H %.0f-%.0f S %.0f-%.0f V %.0f-%.0f",
                H_LOW, H_HIGH, S_LOW, S_HIGH, V_LOW, V_HIGH), 38, white);
        if (best != null) {
            Imgproc.rectangle(frame, best.tl(), best.br(), green, 3);
            putLabel(frame, String.format("Target HSV %.0f %.0f %.0f",
                    tHsv[0], tHsv[1], tHsv[2]), 58, green);
            if (xy != null) {
                putLabel(frame, String.format("Robot x %.1f y %.1f in", xy[0], xy[1]), 78, green);
            }
        }

        Bitmap bmp = Bitmap.createBitmap(w, h, Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, bmp);
        lastFrame.set(bmp);
        return best;
    }

    private static void putLabel(Mat frame, String text, int y, Scalar color) {
        Imgproc.putText(frame, text, new Point(6, y), Imgproc.FONT_HERSHEY_SIMPLEX, 0.55,
                new Scalar(0, 0, 0, 255), 3);
        Imgproc.putText(frame, text, new Point(6, y), Imgproc.FONT_HERSHEY_SIMPLEX, 0.55, color, 1);
    }

    // Pinhole ground-plane projection. Returns {forward, left} from robot center in inches, or null
    // if the pixel is at or above the horizon.
    private static double[] pixelToRobot(double u, double v, int w, int h) {
        double f = (w / 2.0) / Math.tan(Math.toRadians(CAM_H_FOV_DEG) / 2.0);
        double xc = (u - w / 2.0) / f;
        double yc = (v - h / 2.0) / f;
        double tilt = Math.toRadians(CAM_TILT_DOWN_DEG);
        double denom = Math.sin(tilt) + yc * Math.cos(tilt);
        if (denom <= 1e-6) return null;
        double t = (CAM_HEIGHT - TARGET_HEIGHT) / denom;
        double fwd = t * (Math.cos(tilt) - yc * Math.sin(tilt));
        double left = -t * xc;
        double yaw = Math.toRadians(CAM_YAW_DEG);
        double rx = fwd * Math.cos(yaw) - left * Math.sin(yaw) + CAM_FORWARD;
        double ry = fwd * Math.sin(yaw) + left * Math.cos(yaw) + CAM_LEFT;
        return new double[]{rx, ry};
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) { }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    public Point getCenter() { return center; }
    public Rect getRect() { return bestRect; }
    public double[] getTargetHsv() { return targetHsv; }
    public double[] getCenterHsv() { return centerHsv; }
    /** {forward, left} in inches relative to robot center, or null if no target. */
    public double[] getRobotXY() { return robotXY; }
}
