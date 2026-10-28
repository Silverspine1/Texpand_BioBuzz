package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
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

public class pipeline implements VisionProcessor {

    // Tune these for your target color (HSV, OpenCV ranges: H 0-179, S/V 0-255)
    public Scalar lower = new Scalar(40, 87, 80);   // example: blue
    public static Scalar upper = new Scalar(60, 107, 100);
    private final double minArea = 300;
    public static double test;
    private FtcDashboard dashboard;


    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));

    // Results shared with your OpMode thread
    private volatile Rect bestRect = null;
    private volatile Point center = null;

    @Override
    public void init(int width, int height, CameraCalibration calibration) { }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // VisionPortal frames are RGBA
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, lower, upper, mask);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(),
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Rect best = null;
        double bestArea = minArea;
        for (MatOfPoint c : contours) {
            double area = Imgproc.contourArea(c);
            if (area > bestArea) {
                bestArea = area;
                best = Imgproc.boundingRect(c);
            }
            c.release();
        }

        bestRect = best;
        center = (best == null) ? null
                : new Point(best.x + best.width / 2.0, best.y + best.height / 2.0);
        return best; // passed to onDrawFrame as userContext
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        if (userContext instanceof Rect) {
            Rect r = (Rect) userContext;
            android.graphics.Paint p = new android.graphics.Paint();
            p.setStyle(android.graphics.Paint.Style.STROKE);
            p.setStrokeWidth(4 * scaleCanvasDensity);
            p.setColor(android.graphics.Color.GREEN);
            canvas.drawRect(r.x * scaleBmpPxToCanvasPx, r.y * scaleBmpPxToCanvasPx,
                    (r.x + r.width) * scaleBmpPxToCanvasPx,
                    (r.y + r.height) * scaleBmpPxToCanvasPx, p);
        }
    }

    public Point getCenter() { return center; }
    public Rect getRect() { return bestRect; }
}