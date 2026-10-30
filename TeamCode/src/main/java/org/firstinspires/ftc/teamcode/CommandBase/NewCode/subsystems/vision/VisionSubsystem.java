package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.vision;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.Follower.Follower;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathGeneration.PathsManager;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathGeneration.commands.SectionBuilder;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathingUtility.RobotPower;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.RobotUtilities.RobotConfig;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.RobotUtilities.Vector2D;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import dev.weaponboy.nexus_command_base.Commands.LambdaCommand;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;

/**
 * VISION COLLECT subsystem (registered in OpModeEX as {@code vision}).
 *
 * What it does: sees the balls with the intake camera, which is mounted on the BACK intake. It keeps a map of
 * their field positions (so a ball that leaves the picture is still remembered for MEMORY_S), plans the fastest
 * route that sweeps them into the intake, follows it with the Nexus Follower and holds the intake on. The robot
 * reverses into the balls, so the back of the robot leads. It never touches the turret, flywheel or trigger -
 * shooting stays with the driver. It does NOT consider where the route ends (CONSIDER_END_POSITION is off): it
 * just collects as fast as possible.
 *
 * Use it from an OpMode:
 *   initEX():  vision.enable();                       // opens the camera (lazy: other OpModes never open it)
 *   loopEX():  vision.startCollect() / stopCollect();  // a button
 *              if (vision.isCollecting()) { driveBase.drivePowers(vision.getDrivePower()); intake on }
 *
 * ALL TUNABLES ARE THE PUBLIC STATIC FIELDS BELOW (FTC Dashboard > Configuration > VisionSubsystem).
 * Use the "Vision Tuner" OpMode to set them against live camera data, press B there to save them to
 * /sdcard/FIRST/vision_tune.properties (loaded again on every start while USE_TUNED_FILE is true).
 *
 * Units: cm, degrees, seconds. Robot frame: x forward, y LEFT. Odometry frame = whatever odometry reports.
 */
@Config
public class VisionSubsystem extends SubSystem {

    // =====================================================================================================
    // 1. CAMERA  -  where it is and what it sees.  Measure these; they decide every distance it reports.
    // =====================================================================================================
    public static String CAMERA_NAME = "Webcam 1";
    /**
     * Horizontal field of view across the WHOLE captured image, degrees. ELP-USB100W07M-MHV120 is quoted FOV(H) 120 /
     * FOV(D) 150 - that pair only fits a 4:3 picture with an EQUIDISTANT lens (150 = 1.25 x 120), so leave
     * LENS_RECT_MIX at 0. If the camera is run in a 16:9 mode (1280x720) the horizontal FOV may differ: use the
     * Vision Tuner's calibration (see VisionTuner) instead of trusting the number.
     */
    public static double CAM_HFOV_DEG = 120;
    /**
     * Lens shape: 0 = equidistant (angle proportional to pixel radius; the ELP wide-angle lens), 1 = plain pinhole
     * (rectilinear). In between blends them. WRONG VALUE = wrong ball size = one ball split into several detections.
     */
    public static double LENS_RECT_MIX = 0.0;
    /**
     * Camera position from the chassis CENTRE: forward, left (+), up. cm. The camera is on the BACK, so forward is
     * negative. -20 is a placeholder: measure your mount.
     */
    public static double CAM_FORWARD_CM = -20;
    public static double CAM_LEFT_CM = 0;
    public static double CAM_HEIGHT_CM = 22;
    /** Tilt below horizontal, degrees. */
    public static double CAM_PITCH_DOWN_DEG = 50;
    /** Camera yaw, + = turned left, degrees. 180 = camera faces the back of the robot. */
    public static double CAM_YAW_DEG = 180;
    /** Capture size sent by the camera, and the (smaller) size the detector works at. */
    public static int CAPTURE_WIDTH = 640, CAPTURE_HEIGHT = 480;
    public static int WORK_WIDTH = 320;
    /** 0 = same aspect as the capture (never squash the picture: pixels must stay square). */
    public static int WORK_HEIGHT = 0;
    /** Manual exposure (ms) and gain; keep exposure short so moving balls do not smear. */
    public static double EXPOSURE_MS = 10;
    public static double GAIN = 5;
    /** Delay from exposure to the result arriving, s. Used when the SDK capture time is not trusted. */
    public static double CAM_LATENCY_S = 0.08;
    public static boolean USE_CAPTURE_TIME = true;
    /** Ball radius, cm (POLLEN is 2.8 in across). */
    public static double BALL_RADIUS_CM = 3.556;

    // =====================================================================================================
    // 2. COLOUR  -  HSV window for the ball (OpenCV scale: H 0-179, S/V 0-255).  Vision Tuner: press A
    //    with a ball in the centre of the picture to fill these in automatically.
    // =====================================================================================================
    public static double H_LOW = 18, H_HIGH = 38;
    public static double S_LOW = 90, S_HIGH = 255;
    public static double V_LOW = 70, V_HIGH = 255;
    /** Margins the auto-sample adds around the sampled ball colour. */
    public static double AUTO_H_MARGIN = 12, AUTO_S_MARGIN = 70, AUTO_V_MARGIN = 90;

    // =====================================================================================================
    // 3. INTAKE  -  where the intake is relative to the chassis centre, and how picky to be.
    // =====================================================================================================
    /**
     * Chassis centre -> the point where a ball starts being pulled into the intake, cm. NEGATIVE = intake at the back.
     */
    public static double INTAKE_FORWARD_CM = -38.1;
    /** Mouth width, cm (information; the tolerance below is what the planner uses). */
    public static double INTAKE_WIDTH_CM = 44.5;
    /**
     * How far off the intake CENTRE line a ball may be and still be taken, cm. Bigger = it will collect two
     * close balls in one pass and take balls with the side of the intake instead of turning to centre them;
     * smaller = it centres every ball. Keep it a few cm under half the mouth width.
     */
    public static double INTAKE_SIDE_TOLERANCE_CM = 15;
    /** Intake centre line offset from the chassis centre, cm (+ = left). 0 if the intake is centred. */
    public static double INTAKE_OFFSET_CM = 0;
    /**
     * Back intake only: added to the planner's heading target so the back of the robot leads. Units match
     * CollectPlanner.headingTarget (degrees here; set to Math.PI if that method returns radians).
     */
    public static double BACK_INTAKE_HEADING_OFFSET = 180;
    /** Seconds of route time each cm of off-centre costs. 0 = off-centre is free; bigger = prefers centring. */
    public static double CENTER_PREFERENCE_S_PER_CM = 0.005;
    /** Collect this many balls then stop (hopper size). */
    public static int BALLS_TO_COLLECT = 4;
    /**
     * Only go for the balls seen when the collect started, and stop once they are taken, instead of hunting on for
     * more until BALLS_TO_COLLECT. A start ball is followed while its track stays within START_BALL_RADIUS_CM of
     * where it was last seen; balls seen in the first START_LATCH_S count as start balls.
     */
    public static boolean ONLY_BALLS_SEEN_AT_START = true;
    public static double START_BALL_RADIUS_CM = 25;
    public static double START_LATCH_S = 0.25;

    // =====================================================================================================
    // 4. COLLECT BEHAVIOUR  -  speed model the planner uses (match the Follower's real capability) and rules.
    // =====================================================================================================
    public static double MAX_SPEED_CM_S = 180;
    public static double MAX_ACCEL_CM_S2 = 250;
    public static double MAX_TURN_DEG_S = 360;
    /** Fraction of MAX_SPEED the planner assumes on collection legs. */
    public static double CRUISE_FRACTION = 0.9;
    /** Keep driving this far past the last ball, cm. */
    public static double OVERRUN_CM = 6;
    /** A ball closer to a wall than this is approached head-on, cm. */
    public static double WALL_TOUCH_CM = 15;
    /** Keep the chassis centre at least this far from a wall, cm. */
    public static double WALL_MARGIN_CM = 26.7;
    /** How long an unseen ball stays on the map, s. */
    public static double MEMORY_S = 3.0;
    /** Give up (hand control back) after this long with nothing to chase, s. */
    public static double NO_TARGET_TIMEOUT_S = 0.8;
    /** Hard stop for one collect run, s. */
    public static double COLLECT_TIMEOUT_S = 6.0;
    /** Driver stick deflection above which a collect run is cancelled. */
    public static double DRIVER_OVERRIDE_STICK = 0.2;
    /** Leave OFF: the route ignores where it ends (it just collects fastest). */
    public static boolean CONSIDER_END_POSITION = false;

    // =====================================================================================================
    // 5. FRAME  -  odometry handedness.  If a ball to the robot's LEFT is mapped to the wrong side, flip this.
    //    -1 (default): robot-left = (sin h, -cos h) in the odometry frame (what Follower's strafe maths implies);
    //    +1: robot-left = (-sin h, cos h).
    //    This is about odometry, not the camera, so it does not change when the camera moves.
    // =====================================================================================================
    public static int LATERAL_SIGN = -1;

    // =====================================================================================================
    // 6. DISPLAY  -  what the Dashboard camera stream shows (also X on gamepad1 in the Vision Tuner).
    // =====================================================================================================
    /** 0 = camera + circled balls, 1 = yellow MASK, 2 = camera through the mask, 3 = camera | mask side by side. */
    public static int VIEW = 0;
    /** Draw red reference circles of the ball size the camera model expects (put a real ball on one to compare). */
    public static boolean SHOW_EXPECTED_SIZE = false;
    /** Half-size (px of the processed image) of the centre colour-sampling box. */
    public static int CENTER_BOX_PX = 6;

    // =====================================================================================================
    // 7. ADVANCED  -  detector / tracker.  Leave alone unless the Vision Tuner shows a reason.
    // =====================================================================================================
    /**
     * Impossible-overlap filter: two balls cannot overlap, so detections whose centres are closer than
     * DET_SEP_FRAC x 2R on the floor (or DET_IMG_SEP_FRAC x (r1+r2) in the image) cannot both be real - the one that
     * explains less of the mask on its own is dropped. This removes false balls between touching balls.
     */
    public static double DET_SEP_FRAC = 0.80, DET_IMG_SEP_FRAC = 0.55, DET_DUP_FRAC = 0.35, DET_UNIQUE_MIN = 0.04;
    public static double DET_SCORE_MIN = 0.30, DET_NEW_MIN = 0.18, DET_FLOOR_PENALTY = 1.0;
    public static double DET_HOLE_CLOSE_FRAC = 0.30;
    public static double MAX_USE_RANGE_CM = 130;
    public static int CONFIRM_HITS = 2;
    public static double ASSOCIATION_GATE_CM = 5;
    public static double REPLAN_S = 0.25;
    /**
     * false (default): once a path is started it is driven to its end, then the next one is planned. Swapping to a
     * fresh plan mid-path every REPLAN_S made the drive twitchy and sporadic. true = the old behaviour.
     */
    public static boolean REPLAN_WHILE_DRIVING = false;
    /**
     * Heading while collecting = the path's own direction this far ahead along it (the Follower's path-heading mode),
     * plus BACK_INTAKE_HEADING_OFFSET for a back intake. 0 = the old per-leg chord, which jumped at every waypoint.
     */
    public static double PATH_HEADING_LOOKAHEAD_CM = 60;
    public static int PLAN_CANDIDATES = 8;
    public static boolean USE_TUNED_FILE = true;
    public static boolean SEND_DASHBOARD = true;

    // =====================================================================================================
    // implementation
    // =====================================================================================================
    public enum State { IDLE, COLLECTING, DONE }

    public static final String TUNE_FILE = "/sdcard/FIRST/vision_tune.properties";

    private final OpModeEX op;
    private BallVisionProcessor processor;
    private VisionPortal portal;
    private boolean enabled = false, controlsApplied = false;

    private VisionParams params = new VisionParams();
    private BallCamera camera;
    private BallMap map;
    private CollectPlanner planner;
    private double geometryKey = Double.NaN;

    private State state = State.IDLE;
    public String doneReason = "";
    private Follower follow;                       // created on first use (RobotConfig reads the pathing values file)
    private CollectPlanner.Plan plan;
    private double planStartS, lastPlanS, noTargetS, collectStartS, stallS;
    private int ballsTaken;
    private final int[] leg = {1};
    private long lastSeq = -1;
    private double lastTickS = 0, tickDt = 0.02;
    private RobotPower drivePower = null;
    private final java.util.Map<Integer, Double> blacklist = new java.util.HashMap<>();
    private final List<double[]> startBalls = new ArrayList<>();   // {x, y} of the balls seen at the start
    private boolean startLatched;
    private int collectTarget;
    private BallVisionProcessor.Result lastResult;
    private double lastDashboardS;
    public double fps, procMs;

    public final LambdaCommand update = new LambdaCommand(() -> {}, this::tick, () -> false);

    public VisionSubsystem(OpModeEX opModeEX) {
        this.op = opModeEX;
        registerSubsystem(opModeEX, update);
    }

    @Override
    public void init() {
        if (USE_TUNED_FILE) VisionTuneFile.load();
    }

    @Override
    public void execute() {
        executeEX();
    }

    // ---- lifecycle -------------------------------------------------------------------------------------
    /** Open the camera. Safe to call twice. Does nothing if the camera cannot be opened (state stays IDLE). */
    public void enable() {
        if (enabled) return;
        try {
            processor = new BallVisionProcessor();
            portal = buildPortal(op.hardwareMap, processor);
            if (SEND_DASHBOARD) FtcDashboard.getInstance().startCameraStream(processor, 15);
            enabled = true;
        } catch (Exception e) {
            op.telemetry.addData("VISION", "camera failed: " + e.getMessage());
            enabled = false;
        }
    }

    public void disable() {
        stopCollect();
        if (!enabled) return;
        try {
            if (SEND_DASHBOARD) FtcDashboard.getInstance().stopCameraStream();
        } catch (Exception ignored) { }
        if (portal != null) portal.close();
        portal = null;
        enabled = false;
        controlsApplied = false;
    }

    public boolean isEnabled() { return enabled; }

    /** Shared by the Vision Tuner OpMode. */
    public static VisionPortal buildPortal(HardwareMap hw, BallVisionProcessor proc) {
        return new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, CAMERA_NAME))
                .setCameraResolution(new Size(CAPTURE_WIDTH, CAPTURE_HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(proc)
                .build();
    }

    /** Applies manual exposure / gain once the stream is running. Shared by the Vision Tuner. */
    public static boolean applyCameraControls(VisionPortal portal) {
        if (portal == null || portal.getCameraState() != VisionPortal.CameraState.STREAMING) return false;
        try {
            ExposureControl ex = portal.getCameraControl(ExposureControl.class);
            if (ex.getMode() != ExposureControl.Mode.Manual) ex.setMode(ExposureControl.Mode.Manual);
            ex.setExposure((long) EXPOSURE_MS, TimeUnit.MILLISECONDS);
            GainControl g = portal.getCameraControl(GainControl.class);
            g.setGain((int) GAIN);
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    /** Copy every tunable above into a plain-data params object (called every loop, so Dashboard edits apply live). */
    public static VisionParams buildParams() {
        VisionParams v = new VisionParams();
        v.workWidth = WORK_WIDTH;
        v.workHeight = WORK_HEIGHT > 0 ? WORK_HEIGHT
                : Math.max(2, (int) Math.round((double) WORK_WIDTH * CAPTURE_HEIGHT / Math.max(CAPTURE_WIDTH, 1)));
        v.hfovDeg = CAM_HFOV_DEG; v.lensRectMix = Math.max(0, Math.min(1, LENS_RECT_MIX));
        v.camForwardCm = CAM_FORWARD_CM; v.camLeftCm = CAM_LEFT_CM; v.camHeightCm = CAM_HEIGHT_CM;
        v.camPitchDownDeg = CAM_PITCH_DOWN_DEG; v.camYawDeg = CAM_YAW_DEG; v.ballRadiusCm = BALL_RADIUS_CM;
        v.lateralSign = LATERAL_SIGN >= 0 ? 1 : -1;
        v.scoreMin = DET_SCORE_MIN; v.newMin = DET_NEW_MIN; v.floorPenalty = DET_FLOOR_PENALTY; v.sepFrac = DET_SEP_FRAC;
        v.imgSepFrac = DET_IMG_SEP_FRAC; v.dupFrac = DET_DUP_FRAC; v.uniqueMin = DET_UNIQUE_MIN;
        v.holeFrac = DET_HOLE_CLOSE_FRAC; v.maxDetRangeCm = MAX_USE_RANGE_CM;
        v.confirmHits = CONFIRM_HITS; v.gateCm = ASSOCIATION_GATE_CM; v.memoryS = MEMORY_S;
        v.intakeForwardCm = INTAKE_FORWARD_CM; v.intakeWidthCm = INTAKE_WIDTH_CM;
        v.intakeSideTolCm = INTAKE_SIDE_TOLERANCE_CM; v.intakeOffsetCm = INTAKE_OFFSET_CM;
        v.centerCostSPerCm = CENTER_PREFERENCE_S_PER_CM;
        v.cruiseFrac = CRUISE_FRACTION; v.overrunCm = OVERRUN_CM; v.wallMarginCm = WALL_MARGIN_CM; v.wallTouchCm = WALL_TOUCH_CM;
        v.maxSpeedCmS = MAX_SPEED_CM_S; v.maxAccelCmS2 = MAX_ACCEL_CM_S2; v.headingRateDps = MAX_TURN_DEG_S;
        v.replanS = REPLAN_S;
        v.headingLookaheadCm = PATH_HEADING_LOOKAHEAD_CM;
        v.backHeadingOffsetDeg = BACK_INTAKE_HEADING_OFFSET; v.nCandidates = PLAN_CANDIDATES; v.noTargetTimeoutS = NO_TARGET_TIMEOUT_S;
        return v;
    }

    // ---- public API ------------------------------------------------------------------------------------
    public State state() { return state; }

    public boolean isCollecting() { return state == State.COLLECTING; }

    public int ballsTaken() { return ballsTaken; }

    /** Drive command while collecting (field-relative path following); null when not collecting. */
    public RobotPower getDrivePower() { return state == State.COLLECTING ? drivePower : null; }

    /** True while the intake should be running. */
    public boolean wantsIntake() { return state == State.COLLECTING; }

    public BallMap map() { return map; }

    public BallVisionProcessor processor() { return processor; }

    public void startCollect() {
        if (!enabled) enable();
        if (!enabled || camera == null) return;
        state = State.COLLECTING;
        doneReason = "";
        plan = null;
        ballsTaken = 0;
        noTargetS = 0;
        lastPlanS = -1e9;
        collectStartS = nowS();
        blacklist.clear();
        drivePower = null;
        startBalls.clear();
        startLatched = false;
        collectTarget = BALLS_TO_COLLECT;
    }

    public void stopCollect() {
        if (state == State.COLLECTING) { state = State.IDLE; doneReason = "stopped"; }
        plan = null;
        drivePower = null;
    }

    // ---- loop ------------------------------------------------------------------------------------------
    private static double nowS() { return System.nanoTime() * 1e-9; }

    private double[] pose() {
        return new double[]{op.odometry.X(), op.odometry.Y(), op.odometry.Heading()};
    }

    /** Changes whenever anything that needs the camera / detector tables rebuilt changes. Shared with the Vision Tuner. */
    public static double geometryKey(VisionParams v) {
        return java.util.Arrays.hashCode(new double[]{v.workWidth, v.workHeight, v.hfovDeg, v.lensRectMix, v.camForwardCm,
                v.camLeftCm, v.camHeightCm, v.camPitchDownDeg, v.camYawDeg, v.ballRadiusCm, v.lateralSign, v.scoreMin,
                v.newMin, v.floorPenalty, v.sepFrac, v.imgSepFrac, v.dupFrac, v.uniqueMin, v.holeFrac, v.maxDetRangeCm,
                v.radiusScale, v.minRadiusPx});
    }

    private void rebuildIfGeometryChanged() {
        VisionParams v = buildParams();
        double key = geometryKey(v);
        params = v;
        if (camera == null || key != geometryKey) {
            geometryKey = key;
            camera = new BallCamera(v);
            ClusterDetector det = new ClusterDetector(camera);
            BallMap old = map;
            map = new BallMap(camera);
            if (old != null) { /* tracks are re-found within a frame or two */ }
            planner = new CollectPlanner(v);
            if (processor != null) processor.configure(camera, det, v);
        } else {
            planner = new CollectPlanner(v);
            map.setParams(v);
            if (processor != null) processor.setParams(v);
        }
    }

    private void tick() {
        if (!enabled) return;
        if (!controlsApplied) controlsApplied = applyCameraControls(portal);
        rebuildIfGeometryChanged();
        double now = nowS();
        double dt = lastTickS == 0 ? 0.02 : Math.min(now - lastTickS, 0.1);
        lastTickS = now;
        tickDt = dt;
        double[] pose = pose();
        map.pushPose(now, pose[0], pose[1], pose[2]);

        BallVisionProcessor.Result r = processor.latest();
        if (r != null && r.seq != lastSeq) {
            lastSeq = r.seq;
            lastResult = r;
            double expose = USE_CAPTURE_TIME ? r.captureS : r.receivedS - CAM_LATENCY_S;
            if (Math.abs(expose - now) > 1.0) expose = r.receivedS - CAM_LATENCY_S;   // clock mismatch guard
            map.update(expose, r.detections, now);
            fps = r.fps;
            procMs = r.procMs;
        }
        map.pruneAge(now);
        consumeTracksInMouth(pose);

        if (state == State.COLLECTING) collectTick(now, pose);
        else drivePower = null;

        if (SEND_DASHBOARD && now - lastDashboardS > 0.1) {
            lastDashboardS = now;
            sendDashboard(pose);
        }
    }

    /**
     * A ball that has reached the intake mouth counts as taken (no hopper sensor is assumed).
     * Works for a front or back intake: the sign of the intake offset says which way the mouth points.
     */
    private void consumeTracksInMouth(double[] pose) {
        if (state != State.COLLECTING) return;
        double dir = Math.signum(params.intakeForwardCm);      // +1 front intake, -1 back intake
        double reach = Math.abs(params.intakeForwardCm) * 0.55;
        for (BallMap.Track t : map.all()) {
            double[] rel = camera.fieldToRobot(t.x, t.y, pose);
            double along = rel[0] * dir;
            if (along > 6 && along < reach && Math.abs(rel[1] - params.intakeOffsetCm) < params.intakeSideTolCm + 5) {
                map.remove(t.id);
                ballsTaken++;
                int k = nearestStartBall(t.x, t.y, START_BALL_RADIUS_CM);
                if (k >= 0) startBalls.remove(k);
            }
        }
    }

    private void finish(String why) {
        state = State.DONE;
        doneReason = why;
        plan = null;
        drivePower = null;
    }

    private int nearestStartBall(double x, double y, double within) {
        int best = -1;
        double bestD = within;
        for (int i = 0; i < startBalls.size(); i++) {
            double d = Math.hypot(startBalls.get(i)[0] - x, startBalls.get(i)[1] - y);
            if (d < bestD) { bestD = d; best = i; }
        }
        return best;
    }

    /** Remembers the balls in view during the first START_LATCH_S, then keeps their positions current. */
    private void trackStartBalls(double now, double[] pose) {
        if (!ONLY_BALLS_SEEN_AT_START) return;
        List<BallMap.Track> seen = map.usable(pose);
        if (!startLatched) {
            double same = 2 * params.ballRadiusCm + 2;
            for (BallMap.Track t : seen) if (nearestStartBall(t.x, t.y, same) < 0) startBalls.add(new double[]{t.x, t.y});
            if (now - collectStartS >= START_LATCH_S && !startBalls.isEmpty()) {
                startLatched = true;
                collectTarget = Math.min(BALLS_TO_COLLECT, startBalls.size());
            }
        } else {
            for (BallMap.Track t : seen) {
                int k = nearestStartBall(t.x, t.y, START_BALL_RADIUS_CM);
                if (k >= 0) { startBalls.get(k)[0] = t.x; startBalls.get(k)[1] = t.y; }
            }
        }
    }

    private void collectTick(double now, double[] pose) {
        trackStartBalls(now, pose);
        int room = collectTarget - ballsTaken;
        if (room <= 0) { finish(startLatched ? "took the balls seen at start" : "full"); return; }
        if (now - collectStartS > COLLECT_TIMEOUT_S) { finish("timeout"); return; }
        double vx = op.odometry.getXVelocity(), vy = op.odometry.getYVelocity();
        double speed = Math.hypot(vx, vy);

        List<BallMap.Track> tracks = new ArrayList<>();
        for (BallMap.Track t : map.usable(pose)) {
            Double until = blacklist.get(t.id);
            if (until != null && until >= now) continue;
            // once the start balls are known, balls that appear later are not chased
            if (startLatched && nearestStartBall(t.x, t.y, START_BALL_RADIUS_CM) < 0) continue;
            tracks.add(t);
        }
        boolean mayPlan = plan == null || REPLAN_WHILE_DRIVING;
        if (mayPlan && !tracks.isEmpty() && now - lastPlanS >= params.replanS) {
            lastPlanS = now;
            List<CollectPlanner.Plan> cands = planner.planCandidates(pose, tracks, room, params.nCandidates);
            // a committed plan is never compared or swapped (see REPLAN_WHILE_DRIVING)
            CollectPlanner.Plan best = cands.isEmpty() ? null : planner.chooseBest(cands, pose, speed, tracks, room);
            if (best != null && best.nExpected > 0) {
                boolean keep = false;
                if (plan != null) {
                    double age = now - planStartS;
                    double oldLeft = Math.max(plan.tEst - age, 0.05);
                    double oldScore = oldLeft - params.ballValueS * plan.nExpected;
                    double newScore = best.tEst - params.ballValueS * best.nExpected;
                    boolean firstStillThere = false;
                    if (!plan.anchors.isEmpty()) for (BallMap.Track t : tracks) if (t.id == (int) plan.anchors.get(0)[0]) firstStillThere = true;
                    keep = firstStillThere && (newScore > oldScore - params.switchGainS || age < params.minPlanS);
                }
                if (!keep) adopt(best, now);
            }
        }
        if (plan == null) {
            noTargetS += tickDt;
            drivePower = null;
            if (noTargetS >= NO_TARGET_TIMEOUT_S) finish("no_targets");
            return;
        }
        noTargetS = 0;
        double target = CollectPlanner.headingTarget(plan, pose[0], pose[1], leg);
        // Back intake: the robot reverses into the balls, so its back has to lead.
        if (INTAKE_FORWARD_CM < 0) target += BACK_INTAKE_HEADING_OFFSET;
        // with PATH_HEADING_LOOKAHEAD_CM > 0 the follower takes the heading from the path itself (target unused)
        drivePower = follow.followPathAuto(target, pose[2], pose[0], pose[1], vx, vy, op.odometry.getHeadingVelocityDeg());
        double[] end = plan.end();
        double endErr = Math.hypot(end[0] - pose[0], end[1] - pose[1]);
        stallS = (speed < 6.0 && now - planStartS > 0.5) ? stallS + tickDt : 0;
        boolean overTime = now - planStartS > 1.8 * plan.tEst + 1.0;
        if (endErr < params.stopDistCm || stallS > 0.7 || overTime) {
            // balls the map still holds that we drove at were not takeable from here: skip them for a while
            for (int id : plan.collects) {
                for (BallMap.Track t : map.all()) if (t.id == id) blacklist.put(id, now + params.blacklistS);
            }
            plan = null;
            lastPlanS = -1e9;
        }
    }

    private void adopt(CollectPlanner.Plan best, double now) {
        PathsManager paths = new PathsManager(new RobotConfig());
        paths.addNewPath("vision");
        final double[][] bz = best.bezier;
        SectionBuilder[] sections = new SectionBuilder[bz.length];
        for (int i = 0; i < bz.length; i++) {
            final double[] b = bz[i];
            sections[i] = () -> paths.addPoints(new Vector2D(b[0], b[1]), new Vector2D(b[2], b[3]),
                    new Vector2D(b[4], b[5]), new Vector2D(b[6], b[7]));
        }
        paths.buildPath(sections);
        if (follow == null) follow = new Follower(new RobotConfig());
        boolean pathHeadings = PATH_HEADING_LOOKAHEAD_CM > 0;
        follow.usePathHeadings(pathHeadings);
        if (pathHeadings) {
            follow.setHeadingLookAheadDistance((int) Math.round(PATH_HEADING_LOOKAHEAD_CM / 0.25));   // path points are 0.25 cm
            follow.setHeadingOffset(INTAKE_FORWARD_CM < 0 ? BACK_INTAKE_HEADING_OFFSET : 0);
        }
        follow.setPath(paths.returnPath("vision"));
        plan = best;
        planStartS = now;
        stallS = 0;
        leg[0] = 1;
    }

    // ---- telemetry / dashboard -------------------------------------------------------------------------
    public void addTelemetry(Telemetry t) {
        t.addLine("=== VISION COLLECT ===");
        t.addData("state", state + (doneReason.isEmpty() ? "" : " (" + doneReason + ")"));
        if (!enabled) { t.addData("camera", "not enabled"); return; }
        t.addData("camera", portal == null ? "-" : portal.getCameraState());
        t.addData("fps / detect ms", "%.1f / %.1f", fps, procMs);
        if (lastResult != null && !lastResult.detections.isEmpty()) {
            ClusterDetector.Detection n = lastResult.detections.get(0);
            for (ClusterDetector.Detection d : lastResult.detections) if (d.rangeCm() < n.rangeCm()) n = d;
            t.addData("nearest ball", "%.0f cm fwd, %.0f cm left (range %.0f, bearing %.0f deg)", n.xr, n.yr, n.rangeCm(), n.bearingDeg());
        }
        t.addData("seen / tracked", "%d / %d", lastResult == null ? 0 : lastResult.detections.size(), map == null ? 0 : map.size());
        t.addData("balls taken", "%d / %d%s", ballsTaken, collectTarget, startLatched ? " (balls seen at start)" : "");
        if (plan != null) t.addData("plan", "%d balls, %.2f s, %d legs", plan.nExpected, plan.tEst, plan.bezier.length);
    }

    private void sendDashboard(double[] pose) {
        try {
            TelemetryPacket pk = new TelemetryPacket();
            Canvas c = pk.fieldOverlay();
            c.setStroke("#1e88e5");
            double hx = 22, hw = 22;
            double[] xs = new double[4], ys = new double[4];
            double[][] corner = {{hx, hw}, {hx, -hw}, {-hx, -hw}, {-hx, hw}};
            for (int i = 0; i < 4; i++) {
                double[] f = camera.robotToField(corner[i][0], corner[i][1], pose);
                xs[i] = f[0]; ys[i] = f[1];
            }
            c.strokePolygon(xs, ys);
            double[] g = camera.robotToField(params.intakeForwardCm, params.intakeOffsetCm, pose);
            c.setStroke("#43a047").strokeCircle(g[0], g[1], 4);
            c.setStroke("#ffb300");
            for (BallMap.Track t : map.all()) c.strokeCircle(t.x, t.y, params.ballRadiusCm);
            if (plan != null && plan.polyline != null) {
                int n = plan.polyline.length;
                double[] px = new double[n], py = new double[n];
                for (int i = 0; i < n; i++) { px[i] = plan.polyline[i][0]; py[i] = plan.polyline[i][1]; }
                c.setStroke("#e53935").strokePolyline(px, py);
            }
            pk.put("vision state", state.toString());
            pk.put("tracked balls", map.size());
            pk.put("balls taken", ballsTaken);
            FtcDashboard.getInstance().sendTelemetryPacket(pk);
        } catch (Exception ignored) { }
    }
}