package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

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
    public static double CAM_V_FOV_DEG = 88.5;
    public static double CAM_FORWARD = 7.086614;
    public static double CAM_LEFT = 0;
    public static double CAM_YAW_DEG = 0;
    public static double TARGET_HEIGHT = 1.5;

    // BALL RADIUS IN INCHES
    public static double BALL_RADIUS_IN = 1.5;

    // DETECTION SETTINGS
    public static double FLOOR_PENALTY = 1.0;
    // Pixels already claimed by an accepted ball must contribute no positive score to later candidates.
    public static double EXPLAINED_WEIGHT = 1.0;

    public static double SCORE_MIN = 0.15;
    public static double NEW_MIN = 0.20;

    // Fraction of the sum of two image-space ball radii used by adaptive NMS.
    // Lower values preserve more closely overlapping real balls; higher values
    // suppress more duplicate centres from one physical ball.
    public static double DUP_FRAC = 0.55;
    public static double DISTANCE_PEAK_MIN_RADIUS_FRACTION = 0.25;
    public static double MAX_RANGE_IN = 120;

    public static int MAX_BALLS = 10;

    // Temporal validation for detections. A candidate must be seen in
    // consecutive frames before it is exposed to the planner.
    public static int TRACK_CONFIRM_FRAMES = 3;
    public static int TRACK_MAX_MISSED_FRAMES = 2;
    // Matching distance is based on the projected ball radii, so it scales
    // with camera range rather than using a fixed pixel threshold.
    public static double TRACK_MATCH_RADIUS_FACTOR = 0.85;
    public static double TRACK_SMOOTHING = 0.35;

    // Same-frame duplicate suppression, applied before temporal tracking.
    // Candidates closer than this fraction of the larger projected radius
    // are treated as competing centres for the same physical ball. Using the
    // larger radius prevents a small false peak from escaping suppression.
    // Keep below 2.0 so two touching balls can still be represented separately.
    public static double DUPLICATE_CANDIDATE_DISTANCE_FACTOR = 1.65;

    // Changes only the green camera-overlay circle, not triangulation,
    // Detection.radiusPx, robot coordinates, or planner geometry.
    public static double DETECTION_CIRCLE_DISPLAY_SCALE = 1.35;

    // TIME-OPTIMAL PATH PLANNER SETTINGS (FTC Dashboard)
    public static int TARGET_BALL_COUNT = 3;

    // Bézier trajectory fitting and profiling (all distances in inches).
    public static int BEZIER_SAMPLES_PER_SEGMENT = 20;
    public static double MAX_CURVE_DEVIATION_IN = 1.5;
    public static double MAX_PATH_CURVATURE_PER_IN = 0.05;
    public static double MAX_LATERAL_ACCEL_IN_PER_SEC2 = 69.75; // conservative mecanum limit
    public static double PATH_SPEED_SCALE = 0.65;


    // Linear motion constraints, inches and seconds.
    public static double MAX_FORWARD_VEL = 120.4724;
    public static double MAX_REVERSE_VEL = 120.4724;
    public static double MAX_STRAFE_VEL = 79.92126;
    public static double FORWARD_ACCEL = 275.5906;
    public static double REVERSE_ACCEL = 275.5906;
    public static double STRAFE_ACCEL = 177.1654;
    public static double FORWARD_DECEL = 300;
    public static double REVERSE_DECEL = 300;
    public static double STRAFE_DECEL = 200;

    // Angular motion constraints, degrees and seconds.
    public static double MAX_TURN_VEL_DEG = 180.0;
    public static double TURN_ACCEL_DEG = 360.0;
    public static double TURN_DECEL_DEG = 360.0;

    // Time allowed for the intake to acquire one ball.
    public static double BALL_COLLECTION_TIME_SEC = 0.20;

    // FTC Dashboard field-overlay visualization.
    // Dashboard field coordinates use inches with the origin at field centre.
    public static boolean SHOW_PATH_ON_DASHBOARD = true;
    public static boolean SHOW_DETECTIONS_ON_DASHBOARD = true;
    public static double DASHBOARD_ROBOT_MARKER_RADIUS_IN = 4.0;
    public static double DASHBOARD_BALL_MARKER_RADIUS_IN = 1.5;
    public static double DASHBOARD_WAYPOINT_MARKER_RADIUS_IN = 2.5;

    // Distance from robot centre to the front/rear intake mouth (inches).
    // Set these to your measured geometry. Zero makes the waypoint the ball centre.
    public static double FRONT_INTAKE_OFFSET_IN = 0.0;
    public static double REAR_INTAKE_OFFSET_IN = 0.0;

    // Usable side-to-side opening of the intake, measured in inches.
    // IMPORTANT: Set this to the real clear opening width of your intake.
    public static double INTAKE_WIDTH_IN = 14.0;

    // Extra clearance subtracted from each side of the intake opening.
    // This prevents the planner from assuming a ball can be collected
    // when it is barely touching the edge of the opening.
    public static double INTAKE_EDGE_MARGIN_IN = 0.5;

    // Robot field pose. Update these from your localization system.
    // Heading is degrees counter-clockwise from field +X.
    private volatile double robotFieldX = 0.0;
    private volatile double robotFieldY = 0.0;
    private volatile double robotHeadingDeg = 0.0;
    private volatile boolean robotPoseSet = false;

    // Results from the most recent plan.
    private volatile double lastEstimatedPathTimeSec = Double.NaN;
    // 0 = front intake, 1 = rear intake, in route order.
    private volatile int[] lastPathIntakeSides = new int[0];
    // Snapshot of the last planned route for the combined camera/field stream.
    private volatile Point[] lastPlannedPath = new Point[0];
    private volatile double lastBezierTrajectoryTimeSec = Double.NaN;

    /** One sampled trajectory state. Coordinates and distances are inches. */
    public static class TrajectoryPoint {
        public final double xIn, yIn, distanceIn, curvaturePerIn;
        public final double maxVelocityInPerSec, velocityInPerSec, timeSec;

        public TrajectoryPoint(double x, double y, double distance, double curvature,
                               double maxVelocity, double velocity, double time) {
            xIn = x; yIn = y; distanceIn = distance; curvaturePerIn = curvature;
            maxVelocityInPerSec = maxVelocity; velocityInPerSec = velocity; timeSec = time;
        }
    }


    // 0 = camera + overlay
    // 1 = mask
    // 2 = thresholded camera
    public static int VIEW = 0;

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

    // Local maxima from the binary-mask distance transform.
    // Separate maxima are useful seeds for two touching/overlapping balls.
    private static class DistanceSeed {
        int u, v;
        double radiusPx;

        DistanceSeed(int u, int v, double radiusPx) {
            this.u = u;
            this.v = v;
            this.radiusPx = radiusPx;
        }
    }

    // Temporal track for one physical ball. The centre is smoothed across
    // frames so small distance-transform peak changes do not move the target.
    private static class DetectionTrack {
        double u, v, radiusPx, score, coverage, xr, yr;
        int consecutiveHits;
        int missedFrames;

        DetectionTrack(Detection d) {
            u = d.u;
            v = d.v;
            radiusPx = d.radiusPx;
            score = d.score;
            coverage = d.coverage;
            xr = d.xr;
            yr = d.yr;
            consecutiveHits = 1;
        }
    }

    private final List<DetectionTrack> detectionTracks = new ArrayList<>();

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

    // A ball's position along and across a candidate collection sweep.
    private static class BallProjection {
        final int index;
        final double along;
        final double across;

        BallProjection(int index, double along, double across) {
            this.index = index;
            this.along = along;
            this.across = across;
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
                detectBalls(mask, maskData, w, h);

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
                    Math.max(1, (int) Math.round(
                            d.radiusPx * Math.max(
                                    0.10,
                                    Math.min(3.0, DETECTION_CIRCLE_DISPLAY_SCALE)
                            )
                    )),
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

        // Build one side-by-side image: annotated camera on the left,
        // live top-down field, detections and route on the right.
        Mat combinedStream = createCombinedStreamFrame(frame);

        Bitmap bmp = Bitmap.createBitmap(
                combinedStream.cols(),
                combinedStream.rows(),
                Bitmap.Config.RGB_565
        );

        Utils.matToBitmap(combinedStream, bmp);
        combinedStream.release();

        Bitmap old = lastFrame.getAndSet(bmp);

        if (old != null && !old.isRecycled()) {
            old.recycle();
        }

        //frame = mask;

        return foundBalls;
    }

    // ------------------------------------------------------------
    // COMBINED FTC DASHBOARD CAMERA + FIELD STREAM
    // ------------------------------------------------------------

    /**
     * Creates a single bitmap with the processed camera view on the left
     * and a live top-down field visualization on the right.
     *
     * Field coordinates are inches, with (0,0) at field centre.
     */
    private Mat createCombinedStreamFrame(Mat cameraFrame) {
        int cameraWidth = cameraFrame.cols();
        int height = cameraFrame.rows();
        int totalWidth = cameraWidth * 2;

        Mat combined = Mat.zeros(height, totalWidth, cameraFrame.type());

        // Left side: original annotated camera view.
        Mat cameraPane = combined.submat(
                new Rect(0, 0, cameraWidth, height)
        );
        cameraFrame.copyTo(cameraPane);
        cameraPane.release();

        // Right side: dark field panel.
        int panelX = cameraWidth;
        Scalar background = new Scalar(28, 28, 28, 255);
        Imgproc.rectangle(
                combined,
                new Point(panelX, 0),
                new Point(totalWidth - 1, height - 1),
                background,
                -1
        );

        // Fit a 144 x 144 inch field into the right panel.
        double scale = Math.min(
                (cameraWidth - 36.0) / 144.0,
                (height - 64.0) / 144.0
        );
        double centreX = panelX + cameraWidth / 2.0;
        double centreY = height / 2.0;

        // Convert field coordinates (origin at field centre) to image pixels.
        double halfFieldPx = 72.0 * scale;
        Point topLeft = new Point(
                centreX - halfFieldPx,
                centreY - halfFieldPx
        );
        Point bottomRight = new Point(
                centreX + halfFieldPx,
                centreY + halfFieldPx
        );

        Scalar white = new Scalar(235, 235, 235, 255);
        Scalar grid = new Scalar(75, 75, 75, 255);
        Scalar green = new Scalar(0, 220, 80, 255);
        Scalar yellow = new Scalar(0, 220, 255, 255);
        Scalar blue = new Scalar(255, 150, 0, 255);
        Scalar orange = new Scalar(0, 145, 255, 255);

        // Panel title and legend.
        Imgproc.putText(
                combined, "LIVE FIELD + COLLECTION PATH",
                new Point(panelX + 14, 24),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.48, white, 1
        );
        Imgproc.putText(
                combined, "BALLS",
                new Point(panelX + 14, height - 27),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.38, yellow, 1
        );
        Imgproc.putText(
                combined, "FRONT",
                new Point(panelX + 90, height - 27),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.38, green, 1
        );
        Imgproc.putText(
                combined, "REAR",
                new Point(panelX + 175, height - 27),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.38, orange, 1
        );

        // Field grid every 24 inches.
        for (int inches = -48; inches <= 48; inches += 24) {
            double px = fieldToPixelX(inches, centreX, scale);
            double py = fieldToPixelY(inches, centreY, scale);

            Imgproc.line(
                    combined,
                    new Point(px, topLeft.y),
                    new Point(px, bottomRight.y),
                    grid, 1
            );
            Imgproc.line(
                    combined,
                    new Point(topLeft.x, py),
                    new Point(bottomRight.x, py),
                    grid, 1
            );
        }

        // Field boundary and centre markings.
        Imgproc.rectangle(combined, topLeft, bottomRight, white, 2);
        Imgproc.line(
                combined,
                new Point(centreX, topLeft.y),
                new Point(centreX, bottomRight.y),
                grid, 1
        );
        Imgproc.line(
                combined,
                new Point(topLeft.x, centreY),
                new Point(bottomRight.x, centreY),
                grid, 1
        );
        Imgproc.circle(
                combined,
                new Point(centreX, centreY),
                Math.max(2, (int) Math.round(12.0 * scale)),
                grid, 1
        );

        // Draw every detected ball in absolute field coordinates.
        boolean poseAvailable = robotPoseSet;
        double heading = Math.toRadians(robotHeadingDeg);
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        if (SHOW_DETECTIONS_ON_DASHBOARD && poseAvailable) {
            List<Detection> ballSnapshot = getDetections();
            for (int i = 0; i < ballSnapshot.size(); i++) {
                Detection detection = ballSnapshot.get(i);

                double fieldX = robotFieldX
                        + detection.xr * cos - detection.yr * sin;
                double fieldY = robotFieldY
                        + detection.xr * sin + detection.yr * cos;

                Point ballPixel = new Point(
                        fieldToPixelX(fieldX, centreX, scale),
                        fieldToPixelY(fieldY, centreY, scale)
                );

                int radius = Math.max(
                        3,
                        (int) Math.round(DASHBOARD_BALL_MARKER_RADIUS_IN * scale)
                );
                Imgproc.circle(combined, ballPixel, radius, yellow, 2);
                Imgproc.putText(
                        combined,
                        String.valueOf(i + 1),
                        new Point(ballPixel.x + 5, ballPixel.y - 5),
                        Imgproc.FONT_HERSHEY_SIMPLEX,
                        0.35,
                        yellow,
                        1
                );
            }
        }

        // Draw the planned route, starting at the current robot position.
        Point[] route = lastPlannedPath;
        int[] routeSides = lastPathIntakeSides;

        if (poseAvailable && SHOW_PATH_ON_DASHBOARD && route != null) {
            double previousX = robotFieldX;
            double previousY = robotFieldY;

            for (int i = 0; i < route.length; i++) {
                Point waypoint = route[i];
                if (waypoint == null) {
                    continue;
                }

                Point from = new Point(
                        fieldToPixelX(previousX, centreX, scale),
                        fieldToPixelY(previousY, centreY, scale)
                );
                Point to = new Point(
                        fieldToPixelX(waypoint.x, centreX, scale),
                        fieldToPixelY(waypoint.y, centreY, scale)
                );

                // Show the full usable intake capture corridor around this
                // sweep, not just a thin centre-line.
                double segmentDx = waypoint.x - previousX;
                double segmentDy = waypoint.y - previousY;
                double segmentLength = Math.hypot(segmentDx, segmentDy);
                if (segmentLength > 1e-6) {
                    double halfCaptureWidth = Math.max(
                            0.0,
                            Math.max(0.0, INTAKE_WIDTH_IN) * 0.5
                                    - Math.max(0.0, BALL_RADIUS_IN)
                                    - Math.max(0.0, INTAKE_EDGE_MARGIN_IN)
                    );
                    double normalX = -segmentDy / segmentLength;
                    double normalY = segmentDx / segmentLength;
                    Scalar laneColor = new Scalar(110, 75, 0, 255);

                    Point laneStartA = new Point(
                            fieldToPixelX(previousX + normalX * halfCaptureWidth, centreX, scale),
                            fieldToPixelY(previousY + normalY * halfCaptureWidth, centreY, scale)
                    );
                    Point laneEndA = new Point(
                            fieldToPixelX(waypoint.x + normalX * halfCaptureWidth, centreX, scale),
                            fieldToPixelY(waypoint.y + normalY * halfCaptureWidth, centreY, scale)
                    );
                    Point laneStartB = new Point(
                            fieldToPixelX(previousX - normalX * halfCaptureWidth, centreX, scale),
                            fieldToPixelY(previousY - normalY * halfCaptureWidth, centreY, scale)
                    );
                    Point laneEndB = new Point(
                            fieldToPixelX(waypoint.x - normalX * halfCaptureWidth, centreX, scale),
                            fieldToPixelY(waypoint.y - normalY * halfCaptureWidth, centreY, scale)
                    );

                    Imgproc.line(combined, laneStartA, laneEndA, laneColor, 1);
                    Imgproc.line(combined, laneStartB, laneEndB, laneColor, 1);
                }

                Imgproc.line(combined, from, to, blue, 2);

                boolean rear = routeSides != null
                        && i < routeSides.length
                        && routeSides[i] == 1;
                Scalar waypointColor = rear ? orange : green;

                int waypointRadius = Math.max(
                        3,
                        (int) Math.round(
                                DASHBOARD_WAYPOINT_MARKER_RADIUS_IN * scale
                        )
                );
                Imgproc.circle(
                        combined, to, waypointRadius, waypointColor, 2
                );
                Imgproc.putText(
                        combined,
                        rear ? "R" : "F",
                        new Point(to.x + 4, to.y + 12),
                        Imgproc.FONT_HERSHEY_SIMPLEX,
                        0.38,
                        waypointColor,
                        1
                );

                previousX = waypoint.x;
                previousY = waypoint.y;
            }
        }

        // Draw robot body and heading last so it remains visible over the route.
        if (poseAvailable) {
            Point robotPixel = new Point(
                    fieldToPixelX(robotFieldX, centreX, scale),
                    fieldToPixelY(robotFieldY, centreY, scale)
            );

            int robotRadius = Math.max(
                    5,
                    (int) Math.round(
                            DASHBOARD_ROBOT_MARKER_RADIUS_IN * scale
                    )
            );

            Imgproc.circle(
                    combined, robotPixel, robotRadius, green, 2
            );

            double headingEndX = robotFieldX + 10.0 * cos;
            double headingEndY = robotFieldY + 10.0 * sin;

            Imgproc.line(
                    combined,
                    robotPixel,
                    new Point(
                            fieldToPixelX(headingEndX, centreX, scale),
                            fieldToPixelY(headingEndY, centreY, scale)
                    ),
                    green, 3
            );
            Imgproc.putText(
                    combined, "ROBOT",
                    new Point(robotPixel.x + 7, robotPixel.y - 7),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.38, green, 1
            );
        } else {
            Imgproc.putText(
                    combined, "SET ROBOT POSE",
                    new Point(panelX + 30, (int) centreY),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.5, yellow, 1
            );
        }

        // Useful live status.
        Imgproc.putText(
                combined,
                "Balls: " + getDetectionCount()
                        + "  Route: " + (route == null ? 0 : route.length)
                        + "  Time: " + String.format(
                        "%.1f", lastEstimatedPathTimeSec
                ) + "s",
                new Point(panelX + 14, height - 8),
                Imgproc.FONT_HERSHEY_SIMPLEX,
                0.34, white, 1
        );

        return combined;
    }

    private static double fieldToPixelX(
            double fieldX, double centreX, double scale
    ) {
        return centreX + fieldX * scale;
    }

    private static double fieldToPixelY(
            double fieldY, double centreY, double scale
    ) {
        return centreY - fieldY * scale;
    }

    private static Point[] copyPoints(Point[] points) {
        if (points == null || points.length == 0) {
            return new Point[0];
        }

        Point[] copy = new Point[points.length];
        for (int i = 0; i < points.length; i++) {
            Point point = points[i];
            copy[i] = point == null
                    ? null
                    : new Point(point.x, point.y);
        }
        return copy;
    }

    // BUILD RADIUS MAP
    private int[] buildKMap(int w, int h) {

        int[] kmap = new int[w * h];

        // Separate focal lengths are required when horizontal and vertical
        // fields of view are configured independently.
        double fx = (w / 2.0) /
                Math.tan(Math.toRadians(CAM_H_FOV_DEG) / 2.0);
        double fy = (h / 2.0) /
                Math.tan(Math.toRadians(CAM_V_FOV_DEG) / 2.0);

        double tilt = Math.toRadians(CAM_TILT_DOWN_DEG);

        for (int v = 0; v < h; v++) {

            for (int u = 0; u < w; u++) {

                int index = v * w + u;

                double xc = (u - w / 2.0) / fx;
                double yc = (v - h / 2.0) / fy;

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

                // Approximate the pixel radius using the mean focal length.
                double radiusPx =
                        ((fx + fy) * 0.5) * BALL_RADIUS_IN / distance;

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

    // DISTANCE-TRANSFORM PEAKS
    // Each strong local maximum is a possible centre of one physical ball.
    // For two touching balls, the two lobes can produce two separate peaks
    // even though the threshold mask is one connected white region.
    private List<DistanceSeed> buildDistanceSeeds(
            Mat binaryMask,
            int[] kmap,
            int w,
            int h
    ) {
        List<DistanceSeed> seeds = new ArrayList<>();
        Mat distanceMap = new Mat();

        try {
            Imgproc.distanceTransform(
                    binaryMask,
                    distanceMap,
                    Imgproc.DIST_L2,
                    5
            );

            float[] distances = new float[w * h];
            distanceMap.get(0, 0, distances);

            for (int v = 1; v < h - 1; v++) {
                for (int u = 1; u < w - 1; u++) {
                    int index = v * w + u;
                    double radius = distances[index];

                    double expectedRadius = kmap[index];
                    double minPeakRadius = Math.max(
                            2.0,
                            expectedRadius * DISTANCE_PEAK_MIN_RADIUS_FRACTION
                    );
                    if (radius < minPeakRadius) continue;

                    // Keep only 8-neighbour local maxima. A small tolerance
                    // avoids losing peaks to floating-point noise.
                    boolean localMaximum = true;
                    for (int dy = -1; dy <= 1 && localMaximum; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            if (dx == 0 && dy == 0) continue;
                            float neighbour = distances[(v + dy) * w + (u + dx)];
                            if (neighbour > radius + 0.05) {
                                localMaximum = false;
                                break;
                            }
                        }
                    }
                    if (!localMaximum) continue;

                    // Collapse flat/nearby maxima belonging to the same lobe.
                    // This is adaptive in image pixels, so it scales with range.
                    int duplicateIndex = -1;
                    for (int i = 0; i < seeds.size(); i++) {
                        DistanceSeed old = seeds.get(i);
                        double separation = Math.hypot(
                                old.u - u,
                                old.v - v
                        );
                        double suppressionRadius = DUP_FRAC *
                                (old.radiusPx + radius);
                        if (separation < suppressionRadius) {
                            duplicateIndex = i;
                            break;
                        }
                    }

                    if (duplicateIndex < 0) {
                        seeds.add(new DistanceSeed(u, v, radius));
                    } else if (radius > seeds.get(duplicateIndex).radiusPx) {
                        seeds.set(
                                duplicateIndex,
                                new DistanceSeed(u, v, radius)
                        );
                    }
                }
            }
        } finally {
            distanceMap.release();
        }

        return seeds;
    }

    // ADD ONE CIRCLE CANDIDATE AT A DISTANCE-TRANSFORM SEED
    private void addSeedCandidate(
            List<Candidate> candidates,
            DistanceSeed seed,
            byte[] maskData,
            int[] kmap,
            int[] areaLookup,
            int[][] prefM,
            int w,
            int h
    ) {
        int u = seed.u;
        int v = seed.v;
        int index = v * w + u;
        int k = kmap[index];
        int maxK = Math.min(w, h) / 2;

        if (k < 2 || k > maxK) return;
        if (u - k < 0 || u + k >= w) return;
        if (v - k < 0 || v + k >= h) return;
        if ((maskData[index] & 0xFF) == 0) return;

        Candidate c = new Candidate(u, v, k);
        c.area = areaLookup[k];
        c.M = disc(prefM, u, v, k, w, h);
        c.score = (
                c.M - FLOOR_PENALTY * (c.area - c.M)
        ) / (double) c.area;

        double[] xy = pixelToRobot(u + 0.5, v + 0.5, w, h);
        if (xy == null) return;

        c.gx = xy[0];
        c.gy = xy[1];
        candidates.add(c);
    }

    // BUILD CANDIDATES
    // Prefer distance-transform peaks. If the mask is too fragmented or no
    // useful peaks are found, fall back to the original pixel scan.
    private List<Candidate> buildCandidates(
            byte[] maskData,
            int[] kmap,
            int[] areaLookup,
            int[][] prefM,
            List<DistanceSeed> seeds,
            int w,
            int h
    ) {
        List<Candidate> candidates = new ArrayList<>();

        if (seeds != null && !seeds.isEmpty()) {
            for (DistanceSeed seed : seeds) {
                addSeedCandidate(
                        candidates, seed, maskData, kmap,
                        areaLookup, prefM, w, h
                );
            }
            if (!candidates.isEmpty()) return candidates;
        }

        // Original candidate scan retained as a safe fallback.
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
                c.M = disc(prefM, u, v, k, w, h);
                c.score = (
                        c.M - FLOOR_PENALTY * (c.area - c.M)
                ) / (double) c.area;

                double[] xy = pixelToRobot(u + 0.5, v + 0.5, w, h);
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

    // ADAPTIVE IMAGE-SPACE DUPLICATE CHECK
    // Compare centre separation against the sum of projected pixel radii.
    // Unlike a fixed inch threshold, this adapts to camera distance and
    // preserves two genuinely close/touching balls when their centres differ.
    private boolean isDuplicate(
            List<Detection> results,
            double u,
            double v,
            double radiusPx
    ) {
        for (Detection d : results) {
            double separation = Math.hypot(d.u - u, d.v - v);
            double suppressionRadius = DUP_FRAC *
                    (d.radiusPx + radiusPx);
            if (separation < suppressionRadius) {
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
    private List<Detection> detectBallsRaw(
            Mat binaryMask,
            byte[] maskData,
            int w,
            int h
    ) {

        List<Detection> results = new ArrayList<>();

        int[] kmap = buildKMap(w, h);

        int[][] prefM = buildPrefix(maskData, w, h);

        int[] areaLookup =
                buildAreaLookup(Math.min(w, h) / 2);

        List<DistanceSeed> distanceSeeds = buildDistanceSeeds(
                binaryMask, kmap, w, h
        );

        List<Candidate> candidates = buildCandidates(
                maskData,
                kmap,
                areaLookup,
                prefM,
                distanceSeeds,
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
            if (isDuplicate(
                    results,
                    refined.x,
                    refined.y,
                    best.k
            )) {
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

                double pixelDx = c.u - refined.x;
                double pixelDy = c.v - refined.y;
                double pixelDistance = Math.hypot(pixelDx, pixelDy);
                double adaptiveSuppressionRadius = DUP_FRAC *
                        (best.k + c.k);

                if (pixelDistance < adaptiveSuppressionRadius) {
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

    /**
     * Removes competing centres produced inside one physical ball in the
     * same frame. Candidates are considered strongest-first, so a weaker
     * nearby peak cannot create a second temporal track.
     *
     * The threshold uses the larger displayed image radius, including the
     * detection-circle display scale. This is important because the raw
     * projected radius is smaller than the visible ball in the current
     * camera image. The default threshold is 1.65 displayed radii.
     */
    private List<Detection> suppressSameFrameDuplicates(
            List<Detection> detections
    ) {
        List<Detection> ranked = new ArrayList<>(detections);
        Collections.sort(ranked, (a, b) -> {
            int scoreOrder = Double.compare(b.score, a.score);
            if (scoreOrder != 0) return scoreOrder;
            return Double.compare(b.coverage, a.coverage);
        });

        List<Detection> kept = new ArrayList<>();
        double factor = Math.max(
                0.10,
                Math.min(1.75, DUPLICATE_CANDIDATE_DISTANCE_FACTOR)
        );

        for (Detection candidate : ranked) {
            boolean duplicate = false;

            for (Detection accepted : kept) {
                double centreDistance = Math.hypot(
                        candidate.u - accepted.u,
                        candidate.v - accepted.v
                );

                // Use the larger radius and the same scale used by the
                // visible detection circle. The raw projected radius is
                // currently smaller than the physical ball in the camera
                // image, so filtering with the unscaled value was too weak.
                double displayScale = Math.max(
                        0.10,
                        Math.min(3.0, DETECTION_CIRCLE_DISPLAY_SCALE)
                );

                double referenceRadius = Math.max(
                        Math.max(1.0, candidate.radiusPx),
                        Math.max(1.0, accepted.radiusPx)
                ) * displayScale;

                double suppressionDistance = Math.max(
                        4.0,
                        factor * referenceRadius
                );

                if (centreDistance < suppressionDistance) {
                    duplicate = true;
                    break;
                }
            }

            if (!duplicate) {
                kept.add(candidate);
            }
        }

        return kept;
    }

    /**
     * Stabilises de-duplicated detections over time. Each candidate can
     * update at most one track per frame. Unconfirmed tracks are hidden
     * from the planner, preventing transient peaks becoming extra balls.
     */
    private List<Detection> detectBalls(
            Mat binaryMask,
            byte[] maskData,
            int w,
            int h
    ) {
        List<Detection> raw = suppressSameFrameDuplicates(
                detectBallsRaw(binaryMask, maskData, w, h)
        );
        boolean[] trackUpdated = new boolean[detectionTracks.size()];

        // Match the closest valid detection/track pairs first.
        class Match {
            int detectionIndex;
            int trackIndex;
            double distance;
            Match(int d, int t, double distance) {
                detectionIndex = d;
                trackIndex = t;
                this.distance = distance;
            }
        }

        List<Match> matches = new ArrayList<>();
        for (int di = 0; di < raw.size(); di++) {
            Detection d = raw.get(di);
            for (int ti = 0; ti < detectionTracks.size(); ti++) {
                DetectionTrack t = detectionTracks.get(ti);
                double distance = Math.hypot(d.u - t.u, d.v - t.v);
                double allowed = Math.max(4.0,
                        TRACK_MATCH_RADIUS_FACTOR * (d.radiusPx + t.radiusPx));
                if (distance <= allowed) {
                    matches.add(new Match(di, ti, distance));
                }
            }
        }
        Collections.sort(matches, (a, b) ->
                Double.compare(a.distance, b.distance));

        boolean[] detectionUsed = new boolean[raw.size()];
        for (Match match : matches) {
            if (detectionUsed[match.detectionIndex]
                    || trackUpdated[match.trackIndex]) {
                continue;
            }

            Detection d = raw.get(match.detectionIndex);
            DetectionTrack t = detectionTracks.get(match.trackIndex);
            double alpha = Math.max(0.05, Math.min(1.0, TRACK_SMOOTHING));

            t.u += alpha * (d.u - t.u);
            t.v += alpha * (d.v - t.v);
            t.radiusPx += alpha * (d.radiusPx - t.radiusPx);
            t.score = d.score;
            t.coverage = d.coverage;

            // Recalculate robot coordinates from the smoothed image centre.
            double[] xy = pixelToRobot(t.u, t.v, w, h);
            if (xy != null) {
                t.xr = xy[0];
                t.yr = xy[1];
            }

            t.consecutiveHits++;
            t.missedFrames = 0;
            detectionUsed[match.detectionIndex] = true;
            trackUpdated[match.trackIndex] = true;
        }

        // Unmatched detections start provisional tracks. They are not
        // published until TRACK_CONFIRM_FRAMES have been observed.
        for (int di = 0; di < raw.size(); di++) {
            if (detectionUsed[di]) continue;
            if (detectionTracks.size() >= Math.max(MAX_BALLS * 3, MAX_BALLS)) {
                break;
            }
            detectionTracks.add(new DetectionTrack(raw.get(di)));
            boolean[] expanded = new boolean[detectionTracks.size()];
            System.arraycopy(trackUpdated, 0, expanded, 0, trackUpdated.length);
            trackUpdated = expanded;
            trackUpdated[trackUpdated.length - 1] = true;
        }

        // Age tracks that were not observed this frame; discard stale tracks.
        for (int i = detectionTracks.size() - 1; i >= 0; i--) {
            DetectionTrack t = detectionTracks.get(i);
            if (i >= trackUpdated.length || !trackUpdated[i]) {
                t.missedFrames++;
                t.consecutiveHits = 0;
            }
            if (t.missedFrames > Math.max(0, TRACK_MAX_MISSED_FRAMES)) {
                detectionTracks.remove(i);
            }
        }

        List<Detection> stable = new ArrayList<>();
        int confirmFrames = Math.max(1, TRACK_CONFIRM_FRAMES);
        for (DetectionTrack t : detectionTracks) {
            if (t.consecutiveHits >= confirmFrames && t.missedFrames == 0) {
                stable.add(new Detection(
                        t.u, t.v, t.radiusPx, t.score, t.coverage, t.xr, t.yr
                ));
            }
        }
        return stable;
    }

    // PIXEL TO ROBOT COORDINATES
    private static double[] pixelToRobot(
            double u,
            double v,
            int w,
            int h
    ) {

        // Use the independently configured horizontal and vertical FOVs.
        double fx = (w / 2.0) /
                Math.tan(Math.toRadians(CAM_H_FOV_DEG) / 2.0);
        double fy = (h / 2.0) /
                Math.tan(Math.toRadians(CAM_V_FOV_DEG) / 2.0);

        double xc = (u - w / 2.0) / fx;
        double yc = (v - h / 2.0) / fy;

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

    // ------------------------------------------------------------
    // TIME-OPTIMAL FIELD-COORDINATE BALL PATH OPTIMIZER
    // ------------------------------------------------------------

    /**
     * Update from the robot's localization system.
     * Coordinates are inches; heading is degrees CCW from field +X.
     */
    public void setRobotPose(double fieldX, double fieldY, double headingDeg) {
        robotFieldX = fieldX;
        robotFieldY = fieldY;
        robotHeadingDeg = headingDeg;
        robotPoseSet = true;
    }

    /**
     * Returns absolute field-coordinate robot-centre approach waypoints,
     * in collection order. The start pose is not included. Each ball can be collected by either
     * the front or rear intake; the DP chooses the cheaper orientation.
     *
     * Cost model: rotate to a collection heading, then translate using
     * simultaneous mecanum forward/reverse and strafe motion profiles, then
     * add the configured intake acquisition time. Intake offsets adjust the
     * returned robot-centre waypoints; the timing model approximates travel
     * between ball centres. This is an estimate, not a guarantee of real time.
     */
    public Point[] getOptimalPath(int targetCount) {
        lastEstimatedPathTimeSec = Double.NaN;
        lastPathIntakeSides = new int[0];

        if (!robotPoseSet || targetCount <= 0) {
            publishPathOverlay(new Point[0], new int[0]);
            return new Point[0];
        }

        List<Detection> ballSnapshot = getDetections();
        int n = ballSnapshot.size();

        if (n == 0 || targetCount > n || n > MAX_BALLS) {
            publishPathOverlay(new Point[0], new int[0]);
            return new Point[0];
        }

        final double startX = robotFieldX;
        final double startY = robotFieldY;
        final double startHeading = normalizeDegrees(robotHeadingDeg);

        // Convert the detected balls into absolute field coordinates once.
        final Point[] balls = new Point[n];
        for (int i = 0; i < n; i++) {
            Detection d = ballSnapshot.get(i);
            double heading = Math.toRadians(startHeading);
            double fieldX = startX + d.xr * Math.cos(heading) - d.yr * Math.sin(heading);
            double fieldY = startY + d.xr * Math.sin(heading) + d.yr * Math.cos(heading);
            balls[i] = new Point(fieldX, fieldY);
        }

        // A ball is inside the collection lane when its centre is within
        // this distance of the intake centre-line.
        double usableHalfWidth = Math.max(
                0.0,
                Math.max(0.0, INTAKE_WIDTH_IN) * 0.5
                        - Math.max(0.0, BALL_RADIUS_IN)
                        - Math.max(0.0, INTAKE_EDGE_MARGIN_IN)
        );

        boolean[] collected = new boolean[n];
        int collectedCount = 0;

        List<Point> route = new ArrayList<>();
        List<Integer> routeSides = new ArrayList<>();

        double currentX = startX;
        double currentY = startY;
        double currentHeading = startHeading;
        double totalTime = 0.0;

        /*
         * Plan collection sweeps rather than one waypoint per ball.
         *
         * Each candidate is a travel direction. All remaining balls inside
         * the intake-width corridor are treated as one collection pass.
         * The robot therefore drives through a row/cluster of balls instead
         * of aiming its intake centre at each ball individually.
         */
        while (collectedCount < targetCount) {
            List<Double> candidateBearings = new ArrayList<>();

            // Try a line from the current robot position towards each ball.
            for (int i = 0; i < n; i++) {
                if (collected[i]) continue;
                double dx = balls[i].x - currentX;
                double dy = balls[i].y - currentY;
                if (Math.hypot(dx, dy) > 1e-6) {
                    candidateBearings.add(Math.toDegrees(Math.atan2(dy, dx)));
                }
            }

            // Also try directions towards the midpoint of every remaining
            // pair. These are important when balls are side-by-side across
            // the width of the intake.
            for (int i = 0; i < n; i++) {
                if (collected[i]) continue;
                for (int j = i + 1; j < n; j++) {
                    if (collected[j]) continue;

                    double midX = (balls[i].x + balls[j].x) * 0.5;
                    double midY = (balls[i].y + balls[j].y) * 0.5;
                    double dx = midX - currentX;
                    double dy = midY - currentY;

                    if (Math.hypot(dx, dy) > 1e-6) {
                        candidateBearings.add(Math.toDegrees(Math.atan2(dy, dx)));
                    }
                }
            }

            if (candidateBearings.isEmpty()) break;

            double bestAverageTime = Double.POSITIVE_INFINITY;
            double bestPassTime = Double.POSITIVE_INFINITY;
            double bestBearing = 0.0;
            int bestSide = 0;
            double bestEndX = currentX;
            double bestEndY = currentY;
            List<BallProjection> bestGroup = null;

            // Compare both front and rear intake use for every candidate lane.
            for (double bearing : candidateBearings) {
                double radians = Math.toRadians(bearing);
                double dirX = Math.cos(radians);
                double dirY = Math.sin(radians);

                List<BallProjection> laneBalls = new ArrayList<>();

                for (int i = 0; i < n; i++) {
                    if (collected[i]) continue;

                    double dx = balls[i].x - currentX;
                    double dy = balls[i].y - currentY;

                    // Along-track distance and perpendicular distance from
                    // the proposed intake centre-line.
                    double along = dx * dirX + dy * dirY;
                    double across = -dx * dirY + dy * dirX;

                    // Only collect balls in front of the moving intake.
                    if (along > 0.0 && Math.abs(across) <= usableHalfWidth) {
                        laneBalls.add(new BallProjection(i, along, across));
                    }
                }

                if (laneBalls.isEmpty()) continue;

                // The first balls encountered are collected first.
                Collections.sort(laneBalls, (a, b) ->
                        Double.compare(a.along, b.along));

                int passCount = Math.min(
                        targetCount - collectedCount,
                        laneBalls.size()
                );

                double farthestBallAlong = laneBalls.get(passCount - 1).along;

                for (int side = 0; side < 2; side++) {
                    double desiredHeading = collectionHeading(bearing, side);

                    double intakeOffset = side == 1
                            ? Math.max(0.0, REAR_INTAKE_OFFSET_IN)
                            : Math.max(0.0, FRONT_INTAKE_OFFSET_IN);

                    // Stop with the selected intake mouth at the last ball
                    // in this pass, rather than driving the robot centre
                    // directly onto the ball.
                    double driveDistance = Math.max(
                            0.0,
                            farthestBallAlong - intakeOffset
                    );

                    double endX = currentX + dirX * driveDistance;
                    double endY = currentY + dirY * driveDistance;

                    double dx = endX - currentX;
                    double dy = endY - currentY;

                    double turnTime = turnProfileTime(
                            angleDifference(desiredHeading, currentHeading)
                    );
                    double driveTime = translationTime(dx, dy, desiredHeading);
                    double passTime = turnTime + driveTime
                            + safeNonNegative(BALL_COLLECTION_TIME_SEC) * passCount;

                    double averageTime = passTime / passCount;

                    // Prefer the pass with the lowest estimated time per ball.
                    // If tied, prefer collecting more balls in this pass.
                    boolean better = averageTime < bestAverageTime - 1e-6;
                    if (!better && Math.abs(averageTime - bestAverageTime) <= 1e-6
                            && bestGroup != null && passCount > bestGroup.size()) {
                        better = true;
                    }

                    if (better || bestGroup == null) {
                        bestAverageTime = averageTime;
                        bestPassTime = passTime;
                        bestBearing = bearing;
                        bestSide = side;
                        bestEndX = endX;
                        bestEndY = endY;
                        bestGroup = new ArrayList<>(
                                laneBalls.subList(0, passCount)
                        );
                    }
                }
            }

            if (bestGroup == null || bestGroup.isEmpty()
                    || !Double.isFinite(bestPassTime)) {
                break;
            }

            // One waypoint represents the end of a whole collection sweep.
            route.add(new Point(bestEndX, bestEndY));
            routeSides.add(bestSide);
            totalTime += bestPassTime;

            // Mark every ball covered by this intake-width sweep as collected.
            for (BallProjection ball : bestGroup) {
                collected[ball.index] = true;
                collectedCount++;
            }

            currentX = bestEndX;
            currentY = bestEndY;
            currentHeading = collectionHeading(bestBearing, bestSide);
        }

        if (collectedCount < targetCount) {
            // Do not return a misleading partial route when the requested
            // number of balls cannot be covered by the generated sweeps.
            lastEstimatedPathTimeSec = Double.NaN;
            publishPathOverlay(new Point[0], new int[0]);
            return new Point[0];
        }

        Point[] path = route.toArray(new Point[0]);
        int[] intakeSides = new int[routeSides.size()];
        for (int i = 0; i < routeSides.size(); i++) {
            intakeSides[i] = routeSides.get(i);
        }

        lastEstimatedPathTimeSec = totalTime;
        lastPathIntakeSides = intakeSides.clone();

        publishPathOverlay(path, intakeSides);
        return path;
    }

    /** Uses TARGET_BALL_COUNT from FTC Dashboard. */
    public Point[] getOptimalPath() {
        return getOptimalPath(TARGET_BALL_COUNT);
    }

    /** Returns the intake-aware, curvature-limited sampled Bézier path in inches. */
    public Point[] getOptimalBezierPath(int targetCount) {
        Point[] waypoints = getOptimalPath(targetCount);
        if (waypoints == null || waypoints.length == 0 || !robotPoseSet) return new Point[0];
        return buildSafeBezierPath(new Point(robotFieldX, robotFieldY), waypoints);
    }

    public Point[] getOptimalBezierPath() {
        return getOptimalBezierPath(TARGET_BALL_COUNT);
    }

    /** Convert the selected polyline to a constrained Catmull-Rom/Bézier chain. */
    private static Point[] buildSafeBezierPath(Point start, Point[] waypoints) {
        List<Point> anchors = new ArrayList<>();
        anchors.add(new Point(start.x, start.y));
        if (waypoints != null) for (Point p : waypoints) if (p != null) anchors.add(new Point(p.x, p.y));
        if (anchors.size() < 2) return anchors.toArray(new Point[0]);

        int samples = Math.max(4, Math.min(100, BEZIER_SAMPLES_PER_SEGMENT));
        double usableHalfWidth = Math.max(0.0, INTAKE_WIDTH_IN * 0.5
                - BALL_RADIUS_IN - INTAKE_EDGE_MARGIN_IN);
        double deviationLimit = Math.min(Math.max(0.0, MAX_CURVE_DEVIATION_IN),
                usableHalfWidth > 0 ? usableHalfWidth * 0.35 : 0.0);
        List<Point> output = new ArrayList<>();
        output.add(new Point(anchors.get(0).x, anchors.get(0).y));

        for (int i = 0; i < anchors.size() - 1; i++) {
            Point p0 = anchors.get(Math.max(0, i - 1));
            Point p1 = anchors.get(i);
            Point p2 = anchors.get(i + 1);
            Point p3 = anchors.get(Math.min(anchors.size() - 1, i + 2));
            Point rawC1 = new Point(p1.x + (p2.x - p0.x) / 6.0,
                    p1.y + (p2.y - p0.y) / 6.0);
            Point rawC2 = new Point(p2.x - (p3.x - p1.x) / 6.0,
                    p2.y - (p3.y - p1.y) / 6.0);
            List<Point> accepted = null;
            // Reduce tangent strength until deviation and curvature are acceptable.
            for (int attempt = 0; attempt <= 8; attempt++) {
                double scale = Math.pow(0.5, attempt);
                Point c1 = new Point(p1.x + (rawC1.x - p1.x) * scale,
                        p1.y + (rawC1.y - p1.y) * scale);
                Point c2 = new Point(p2.x + (rawC2.x - p2.x) * scale,
                        p2.y + (rawC2.y - p2.y) * scale);
                List<Point> candidate = sampleBezier(p1, c1, c2, p2, samples);
                if (isCurveAcceptable(candidate, p1, p2, deviationLimit,
                        Math.max(0.0001, MAX_PATH_CURVATURE_PER_IN))) {
                    accepted = candidate;
                    break;
                }
            }
            // Safe fallback: straight interpolation cannot cut outside this segment.
            if (accepted == null) accepted = sampleBezier(p1, p1, p2, p2, samples);
            for (Point q : accepted) output.add(q);
        }
        return output.toArray(new Point[0]);
    }

    private static List<Point> sampleBezier(Point p0, Point c1, Point c2, Point p3, int samples) {
        List<Point> points = new ArrayList<>();
        for (int j = 1; j <= samples; j++) {
            double t = j / (double) samples, u = 1.0 - t;
            double b0 = u*u*u, b1 = 3*u*u*t, b2 = 3*u*t*t, b3 = t*t*t;
            points.add(new Point(b0*p0.x+b1*c1.x+b2*c2.x+b3*p3.x,
                    b0*p0.y+b1*c1.y+b2*c2.y+b3*p3.y));
        }
        return points;
    }

    private static boolean isCurveAcceptable(List<Point> points, Point a, Point b,
                                             double maxDeviation, double maxCurvature) {
        for (Point p : points) {
            if (distanceToSegment(p, a, b) > maxDeviation + 1e-6) return false;
        }
        for (int i = 1; i < points.size() - 1; i++) {
            double curvature = curvature(points.get(i-1), points.get(i), points.get(i+1));
            if (!Double.isFinite(curvature) || curvature > maxCurvature) return false;
        }
        return true;
    }

    private static double distanceToSegment(Point p, Point a, Point b) {
        double dx=b.x-a.x, dy=b.y-a.y, len2=dx*dx+dy*dy;
        if (len2 < 1e-12) return Math.hypot(p.x-a.x, p.y-a.y);
        double t=((p.x-a.x)*dx+(p.y-a.y)*dy)/len2;
        t=Math.max(0,Math.min(1,t));
        return Math.hypot(p.x-(a.x+t*dx), p.y-(a.y+t*dy));
    }

    private static double curvature(Point a, Point b, Point c) {
        double ab=Math.hypot(b.x-a.x,b.y-a.y), bc=Math.hypot(c.x-b.x,c.y-b.y), ac=Math.hypot(c.x-a.x,c.y-a.y);
        double denom=ab*bc*ac;
        if (denom < 1e-9) return 0.0;
        double cross=Math.abs((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x));
        return 2.0*cross/denom;
    }

    /** Build a time-parameterized profile over the constrained Bézier samples. */
    public TrajectoryPoint[] getOptimalBezierTrajectory(int targetCount) {
        Point[] path = getOptimalBezierPath(targetCount);
        if (path == null || path.length < 2) { lastBezierTrajectoryTimeSec=Double.NaN; return new TrajectoryPoint[0]; }
        int n=path.length;
        double[] ds=new double[n], k=new double[n], limit=new double[n], v=new double[n], time=new double[n];
        double maxV=Math.max(1e-6, Math.min(MAX_FORWARD_VEL, Math.min(MAX_REVERSE_VEL, MAX_STRAFE_VEL))*Math.max(0.05,Math.min(1.0,PATH_SPEED_SCALE)));
        double accel=Math.max(1e-6,Math.min(FORWARD_ACCEL,Math.min(REVERSE_ACCEL,STRAFE_ACCEL)));
        double decel=Math.max(1e-6,Math.min(FORWARD_DECEL,Math.min(REVERSE_DECEL,STRAFE_DECEL)));
        double lateral=Math.max(1e-6,MAX_LATERAL_ACCEL_IN_PER_SEC2);
        for(int i=1;i<n;i++) ds[i]=ds[i-1]+Math.hypot(path[i].x-path[i-1].x,path[i].y-path[i-1].y);
        for(int i=0;i<n;i++) {
            if(i>0 && i<n-1) k[i]=curvature(path[i-1],path[i],path[i+1]);
            double curveLimit=k[i]>1e-8?Math.sqrt(lateral/k[i]):maxV;
            limit[i]=Math.min(maxV,curveLimit);
            v[i]=limit[i];
        }
        v[0]=0; v[n-1]=0;
        for(int i=1;i<n;i++) v[i]=Math.min(v[i],Math.sqrt(Math.max(0,v[i-1]*v[i-1]+2*accel*(ds[i]-ds[i-1]))));
        for(int i=n-2;i>=0;i--) v[i]=Math.min(v[i],Math.sqrt(Math.max(0,v[i+1]*v[i+1]+2*decel*(ds[i+1]-ds[i]))));
        for(int i=1;i<n;i++) {
            double segment=ds[i]-ds[i-1], sum=v[i]+v[i-1];
            double dt=sum>1e-8?2*segment/sum:0;
            time[i]=time[i-1]+dt;
        }
        TrajectoryPoint[] result=new TrajectoryPoint[n];
        for(int i=0;i<n;i++) result[i]=new TrajectoryPoint(path[i].x,path[i].y,ds[i],k[i],limit[i],v[i],time[i]);
        lastBezierTrajectoryTimeSec=time[n-1];
        return result;
    }

    public TrajectoryPoint[] getOptimalBezierTrajectory() {
        return getOptimalBezierTrajectory(TARGET_BALL_COUNT);
    }

    public double getLastBezierTrajectoryTimeSec() { return lastBezierTrajectoryTimeSec; }

    /**
     * Draw the current robot pose, detected balls, planned route, and intake
     * choices in FTC Dashboard's Field view. Field X/Y must use inches with
     * the field centre as (0,0), matching Dashboard's field-overlay frame.
     * This is called automatically by getOptimalPath().
     */
    private void publishPathOverlay(Point[] path, int[] intakeSides) {
        lastPlannedPath = copyPoints(path);
        if (intakeSides != null) {
            lastPathIntakeSides = intakeSides.clone();
        } else {
            lastPathIntakeSides = new int[0];
        }

        if (!SHOW_PATH_ON_DASHBOARD) return;

        TelemetryPacket packet = new TelemetryPacket();
        com.acmerobotics.dashboard.canvas.Canvas overlay = packet.fieldOverlay();

        // Draw detected balls using the same robot pose used by the planner.
        if (SHOW_DETECTIONS_ON_DASHBOARD && robotPoseSet) {
            double heading = Math.toRadians(robotHeadingDeg);
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);
            overlay.setStroke("#FFFFFF");
            overlay.setStrokeWidth(2);
            for (Detection detection : getDetections()) {
                double fieldX = robotFieldX + detection.xr * cos - detection.yr * sin;
                double fieldY = robotFieldY + detection.xr * sin + detection.yr * cos;
                overlay.strokeCircle(fieldX, fieldY, DASHBOARD_BALL_MARKER_RADIUS_IN);
            }
        }

        // Draw robot position and its current heading.
        if (robotPoseSet) {
            overlay.setFill("#FFFFFF");
            overlay.fillCircle(robotFieldX, robotFieldY, DASHBOARD_ROBOT_MARKER_RADIUS_IN);
            double heading = Math.toRadians(robotHeadingDeg);
            overlay.setStroke("#FFFFFF");
            overlay.setStrokeWidth(2);
            overlay.strokeLine(
                    robotFieldX, robotFieldY,
                    robotFieldX + 9.0 * Math.cos(heading),
                    robotFieldY + 9.0 * Math.sin(heading));
        }

        // Draw route segments from the robot through each waypoint.
        if (path != null && path.length > 0) {
            double fromX = robotPoseSet ? robotFieldX : path[0].x;
            double fromY = robotPoseSet ? robotFieldY : path[0].y;
            overlay.setStroke("#00BFFF");
            overlay.setStrokeWidth(2);

            Point[] displayCurve = robotPoseSet
                    ? buildSafeBezierPath(new Point(robotFieldX, robotFieldY), path)
                    : path;
            for (int i = 1; i < displayCurve.length; i++) {
                overlay.strokeLine(displayCurve[i - 1].x, displayCurve[i - 1].y,
                        displayCurve[i].x, displayCurve[i].y);
            }
            for (int i = 0; i < path.length; i++) {
                Point waypoint = path[i];
                boolean rearIntake = intakeSides != null
                        && i < intakeSides.length && intakeSides[i] == 1;
                String markerColor = rearIntake ? "#FF9900" : "#00DD66";
                overlay.setStroke(markerColor);
                overlay.setFill(markerColor);
                overlay.setStrokeWidth(2);
                overlay.strokeCircle(waypoint.x, waypoint.y, DASHBOARD_WAYPOINT_MARKER_RADIUS_IN);
                overlay.fillCircle(waypoint.x, waypoint.y, 0.9);

                fromX = waypoint.x;
                fromY = waypoint.y;
            }
        }

        packet.put("pathWaypointCount", path == null ? 0 : path.length);
        packet.put("estimatedPathTimeSec", lastEstimatedPathTimeSec);
        if (intakeSides != null) {
            for (int i = 0; i < intakeSides.length; i++) {
                packet.put("pathIntake" + i, intakeSides[i] == 0 ? "FRONT" : "REAR");
            }
        }
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    /** Estimated seconds for the most recently generated route. */
    public double getLastEstimatedPathTimeSec() {
        return lastEstimatedPathTimeSec;
    }

    /** Intake selection for the most recent route: 0=front, 1=rear. */
    public int[] getLastPathIntakeSides() {
        return lastPathIntakeSides.clone();
    }

    /** Robot-centre waypoint that places the selected intake mouth at the ball. */
    private static Point getApproachPoint(Point ball, double fromX, double fromY, int side) {
        double dx = ball.x - fromX;
        double dy = ball.y - fromY;
        double distance = Math.hypot(dx, dy);
        if (distance <= 1e-9) return new Point(ball.x, ball.y);

        double configuredOffset = side == 1
                ? REAR_INTAKE_OFFSET_IN : FRONT_INTAKE_OFFSET_IN;
        double offset = Math.max(0.0, configuredOffset);
        // Never place the robot-centre waypoint beyond its current position.
        offset = Math.min(offset, distance);
        return new Point(
                ball.x - (dx / distance) * offset,
                ball.y - (dy / distance) * offset);
    }

    private static double collectionHeading(double bearingDeg, int side) {
        return normalizeDegrees(bearingDeg + (side == 1 ? 180.0 : 0.0));
    }

    private static double normalizeDegrees(double degrees) {
        double result = degrees % 360.0;
        if (result > 180.0) result -= 360.0;
        if (result < -180.0) result += 360.0;
        return result;
    }

    private static double angleDifference(double targetDeg, double currentDeg) {
        return normalizeDegrees(targetDeg - currentDeg);
    }

    private static double safeNonNegative(double value) {
        return Double.isFinite(value) && value > 0.0 ? value : 0.0;
    }

    /** Time for an accelerate-cruise-decelerate angular profile. */
    private static double turnProfileTime(double angleDeg) {
        return motionProfileTime(
                Math.abs(angleDeg),
                MAX_TURN_VEL_DEG,
                TURN_ACCEL_DEG,
                TURN_DECEL_DEG);
    }

    /**
     * Mecanum translation model: forward/reverse and strafe axes move
     * simultaneously. The slower axis determines leg duration.
     */
    private static double translationTime(double worldDx, double worldDy,
                                          double headingDeg) {
        double heading = Math.toRadians(headingDeg);
        double forward = worldDx * Math.cos(heading) + worldDy * Math.sin(heading);
        double left = -worldDx * Math.sin(heading) + worldDy * Math.cos(heading);

        double forwardTime;
        if (forward >= 0.0) {
            forwardTime = motionProfileTime(forward, MAX_FORWARD_VEL,
                    FORWARD_ACCEL, FORWARD_DECEL);
        } else {
            forwardTime = motionProfileTime(-forward, MAX_REVERSE_VEL,
                    REVERSE_ACCEL, REVERSE_DECEL);
        }

        double strafeTime = motionProfileTime(Math.abs(left), MAX_STRAFE_VEL,
                STRAFE_ACCEL, STRAFE_DECEL);
        return Math.max(forwardTime, strafeTime);
    }

    /**
     * 1D trapezoidal/triangular motion profile from rest to rest.
     * If the distance is too short to reach max velocity, uses a triangle.
     */
    private static double motionProfileTime(double distance, double maxVelocity,
                                            double acceleration, double deceleration) {
        distance = Math.abs(distance);
        if (distance <= 1e-9) return 0.0;
        if (!Double.isFinite(maxVelocity) || !Double.isFinite(acceleration)
                || !Double.isFinite(deceleration)
                || maxVelocity <= 0.0 || acceleration <= 0.0 || deceleration <= 0.0) {
            return Double.POSITIVE_INFINITY;
        }

        double accelDistance = maxVelocity * maxVelocity / (2.0 * acceleration);
        double decelDistance = maxVelocity * maxVelocity / (2.0 * deceleration);

        if (accelDistance + decelDistance <= distance) {
            double cruiseDistance = distance - accelDistance - decelDistance;
            return maxVelocity / acceleration
                    + maxVelocity / deceleration
                    + cruiseDistance / maxVelocity;
        }

        // Peak speed for a triangular profile.
        double peakVelocity = Math.sqrt(
                (2.0 * distance * acceleration * deceleration)
                        / (acceleration + deceleration));
        return peakVelocity / acceleration + peakVelocity / deceleration;
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
