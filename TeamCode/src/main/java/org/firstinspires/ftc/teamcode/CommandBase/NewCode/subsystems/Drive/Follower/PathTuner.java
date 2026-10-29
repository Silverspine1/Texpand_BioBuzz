package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Follower;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathLog;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathingValues;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.Follower.Follower;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathGeneration.PathsManager;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathGeneration.commands.SectionBuilder;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathingUtility.RobotPower;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.RobotUtilities.RobotConfig;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.RobotUtilities.Vector2D;

/**
 * Fully self-tuning path tuner. NOTHING to set - every threshold, tolerance
 * and gain is derived from on-robot measurement.
 *
 * Pipeline:
 *   1. NOISE     - robot sits still; measures odometry position/velocity noise.
 *                  Sets the movement-detect and "stopped" thresholds.
 *   2. BREAKAWAY - per axis, ramps power slowly until the robot actually moves.
 *                  Gives static-friction power kS, and the "position quantum"
 *                  (how far a breakaway lurch travels) which becomes the stop
 *                  tolerance. This is what kills endpoint twitch: endpoint Kp
 *                  is capped so power inside tolerance stays below kS.
 *   3. MAX V/A   - per axis, full power until velocity plateaus. Max velocity
 *                  from the plateau; accel-per-power from a LEAST-SQUARES fit
 *                  of the whole ramp (no single-sample derivatives -> no
 *                  noise-spike gains), plus a first-order + dead-time fit of
 *                  the response (delay caps the heading gains). Then full
 *                  reverse through DriveBase's braking-current ramp (no shock
 *                  on the bare motors) and the decel is measured. Path
 *                  accel/decel is capped to that, with margin.
 *                  Measured values are checked against the MIN/MAX speed and
 *                  accel bounds below (clamped if outside).
 *   4. SEARCH    - per axis, minimising a run cost (time-weighted soft error,
 *                  direction reversals and command chatter once in the area).
 *                  Translation: golden section on the final-approach stiffness
 *                  kEnd, then the speed-feedback gain. Heading: golden section
 *                  on ts, then damping and endpoint softness. Then a confirm run.
 *   5. VERIFY    - battery of paths with the locked values, logged to CSV.
 *
 * Results go into the global PathingValues (and its file), which every
 * new RobotConfig() starts from. Translation is driven from the measured
 * drive model (speed per power, response time, delay, kS, brake decel) -
 * see Pathing.Follower.Follower. Heading uses pole-placement PD (ModelTuner),
 * capped to a safe phase margin on the measured turn response.
 *
 * OUTPUT MODE (Y button): what POWER_LIMIT is saved as -
 *   AS TESTED  - the test power;   FULL POWER - 1.0.
 * The drive model is per unit power, so nothing else needs scaling.
 *
 * Place the robot mid-field (180,180) facing 0, dpad_up to start.
 * Outputs: /sdcard/FIRST/datalogs/tune_best.txt, tune_paths.csv.
 * Controls: dpad_up = start, B = stop & save, X = skip current step,
 *           Y = toggle output mode.
 * Before start: dpad left/right = test power, bumpers = tuning area size (cm).
 */
@TeleOp(name = "PathTuner (self tuning)", group = "tuning")
public class PathTuner extends OpModeEX {

    // ---- internal constants (measurement procedure, not tuning knobs) ----
    private static final double NOISE_TIME      = 1.0;   // s standstill sampling
    private static final double RAMP_RATE       = 0.25;  // power/s breakaway ramp
    private static final double RAMP_MAX_POWER  = 0.8;   // give up above this
    private static final double LURCH_TIME      = 0.35;  // s to watch post-breakaway drift
    private static final double MAXVA_MAX_TIME  = 1;   // s full-power cap
    private static final double BRAKE_GUESS     = 400;   // cm/s^2 assumed brake decel before it is measured
    private static final int    PLATEAU_N       = 5;
    private static final double PLATEAU_TOL     = 0.04;
    private static final double BRAKE_MAX_TIME  = 1.5;   // s
    private static final double PROFILE_MARGIN  = 0.85;  // path accel/decel = this x what braking alone achieves
    private static final double SETTLE_TIMEOUT  = 3.5;   // s per probe run
    private static final double HOLD_WATCH      = 0.8;   // s watched after settling, to catch endpoint twitch
    private static final double RETURN_TIMEOUT  = 6.0;   // s per return attempt
    private static final int    RETURN_RETRIES  = 3;     // rebuild+retry if not home
    private static final double HOME_OK_DIST    = 30;    // cm - close enough to start a path
    private static final int    SETTLE_N        = 5;     // consecutive in-zone+stopped samples
    private static final int    OVERSHOOT_N     = 4;     // consecutive past-tol samples = real
    private static final double SAMPLE_DT_MIN   = 0.005;  // ~every loop
    private static final double PUSH_START_IN   = 30;     // cm inside the box edge the forward / strafe push starts from
    private static final double TAU_MIN = 0.05, TAU_MAX = 0.60;  // s - a fit outside this is a bad fit, not a drivetrain
    private static final double MODEL_R2_MIN = 0.75;             // regression fit accepted above this

    // ======================= WHAT ACCURACY YOU WANT =======================
    // Pick with the A button before starting (or change DEFAULT_PRESET).
    // tol: error that counts as "there" - free in the tuning score, and the
    //      follower stops correcting inside it (hold). Bigger = faster tune.
    // overshoot: how far past the target the follower plans its stop, so it
    //      can brake later (it fades out right at the target).
    //                                     tol cm, tol deg, overshoot cm, overshoot deg
    private static final String[] PRESET_NAMES = {"PRECISE", "BALANCED", "FAST"};
    private static final double[][] PRESETS = {
            {1.5, 1.5, 1.0, 1.0},
            {3.0, 3.0, 3.0, 3.0},
            {5.0, 5.0, 5.0, 5.0}};
    private static final int DEFAULT_PRESET = 1;
    private int preset = DEFAULT_PRESET;
    // ========================================================================

    // ---- run cost: integral of a soft error penalty over a fixed window ----
    //   f(e) = 1 - exp(-(max(0, e - tol) / BALANCE)^2)   (tol from the preset)
    // ~1 per second while far away, ~0.6 at BALANCE, ~0.2 at half of it, ~0
    // when spot on. Getting into the area fast dominates the score; finishing
    // precisely still earns a little; overshoot, hunting and endpoint twitch
    // all show up as error, so no hard tolerance line decides anything.
    // Smaller BALANCE = values a precise final settle more.
    private static final double BALANCE_T = 5.0, BALANCE_H = 5.0;   // cm, deg
    private static final double COST_HORIZON = 4.4;              // s - same window for every run
    private static final double W_TWITCH = 0.08;                 // s per direction reversal once in the area
    private static final double W_CHATTER = 0.1;                 // per unit of command change (all axes) once in the area (buzz / shudder)
    private static final double W_OFF_AXIS = 0.5;                // off-axis error (sideways / heading) as a share of the main penalty
    private static final double COST_LEFT_BOX = 20;

    // ---- search per axis: golden section on the final-approach stiffness
    //      kEnd (log), then the speed-feedback gain (2 tries + parabola
    //      vertex), then a confirm run. Heading the same with turn values. ----
    private static final double KEND_LO = 4, KEND_HI = 40;       // HI is lowered to just above the delay stability limit
    private static final double[] VG_TRY = {0.5, 2.0};
    private static final double VG_LO = 0.25, VG_HI = 4;
    private static final int    GS_EVALS = 5;
    private static final double GOLD = 0.6180339887;
    private static final int    MAX_SAMPLES     = 256;

    // field / safety
    private static final double CX = 180, CY = 180;

    // operator-adjustable (IDLE only): test power and tuning-area side length
    private double maxPower = 0.7;
    private double areaSize = 150;   // cm, square centred on (CX, CY)
    private static final double POWER_STEP = 0.05, POWER_MIN = 0.2;
    private static final double AREA_STEP = 10, AREA_MIN = 120, AREA_MAX = 340;

    // Output mode, toggled with Y: false = values as tested, true = scaled to full power
    private boolean fullPowerOutput = false;

    // ---- sanity bounds on MEASURED speed / accel ----
    // These are FULL-POWER (1.0) equivalents, so they stay valid whatever test
    // power you pick. A measurement outside the range gets clamped to the
    // nearest bound and flagged in telemetry. Tighten them around your real
    // robot's numbers (roughly +/-25% of what you expect).
    // 504 rpm: with ~96 mm wheels the free speed is ~250 cm/s, mecanum
    // forward a bit under that, strafe lower.
    private static final double MIN_V_T = 140,  MAX_V_T = 340;    // cm/s
    private static final double MIN_V_H = 200,  MAX_V_H = 1000;   // deg/s
    private static final double MIN_A_T = 200, MAX_A_T = 4000;   // cm/s^2 (at power 1.0)
    private static final double MIN_A_H = 200, MAX_A_H = 6000;   // deg/s^2 (at power 1.0)

    // ---- measured values (all start invalid, filled by the phases) ----
    // noise floors
    private double posNoise = 0, hdgNoise = 0;          // cm / deg standstill drift
    private double velNoiseT = 0, velNoiseH = 0;        // cm/s / deg/s standstill velocity
    private double moveThreshT, moveThreshH;            // displacement = real movement
    private double stopEpsT, stopEpsH;                  // velocity = actually stopped
    // breakaway
    private final double[] ks = new double[3];          // FWD, STR, TURN breakaway power
    private final double[] quantum = new double[3];     // lurch travel (cm, cm, deg)
    // max v / a
    private final double[] maxV = new double[3];        // FWD, STR, TURN
    private final double[] maxA = new double[3];        // regression slope, per unit power
    private final boolean[] plateauOk = new boolean[3];
    private final boolean[] vClamped = new boolean[3];  // measurement hit a speed bound
    private final boolean[] aClamped = new boolean[3];  // measurement hit an accel bound
    private final double[] brakeDecel = new double[3];  // 0-power (BRAKE) decel, cm/s^2 or deg/s^2
    private final java.util.ArrayList<double[]> brakeSamples = new java.util.ArrayList<>();
    private double brakeV0 = 0;
    private boolean brakeCoast = false;
    // step response fit of each push: dead time, time constant, speed (at test power)
    private final double[] stepDelay = new double[3], stepTau = new double[3], stepV = new double[3];
    private final java.util.ArrayList<double[]> posSamples = new java.util.ArrayList<>();   // {t, displacement}
    // every loop of the push + brake: {t, applied axis power, signed axis speed}
    private final java.util.ArrayList<double[]> modelSamples = new java.util.ArrayList<>();
    private final double[] fitTau = new double[3], fitSpp = new double[3], fitR2 = new double[3];
    private final boolean[] fitOk = new boolean[3];
    private double pushOnT = Double.NaN, pushLastH = 0, pushTurned = 0;
    // yaw rate while pushing straight (FWD, STR) -> side imbalance
    private final double[] pushYaw = new double[2];
    private final boolean[] pushYawOk = new boolean[2];
    private double trimLrMeasured = 0, trimFbMeasured = 0;

    // last drive command (fwd, strafe, turn) and its chatter once in the area
    private double cmdF = 0, cmdS = 0, cmdT = 0;
    private double chatter = 0, chatterLast = Double.NaN;
    private double lastCmdF, lastCmdS, lastCmdT, costOff = 0, lastOffT = 0;

    // control loop period, EMA (s)
    private double loopDt = 0.02;
    private long lastLoopNanos = 0;

    // ---- axes ----
    private enum Ax { FWD, STR, TURN }                   // FWD=robot Y, STR=robot X
    private int axIdx = 0;                               // current axis in phase sequences

    // ---- states ----
    private enum State {
        IDLE, NOISE,
        BREAK_RAMP, BREAK_LURCH, BREAK_RETURN,
        MAXVA_PREP, MAXVA_PUSH, MAXVA_BRAKE, MAXVA_RETURN,
        SEARCH_PROBE, SEARCH_RETURN,
        VERIFY_RUN, VERIFY_RETURN,
        FINISHED
    }
    private State state = State.IDLE;

    // ---- pathing objects ----
    private RobotConfig genConfig;
    private PathsManager paths;
    private Follower follow;        // tuned gains
    private Follower safeFollow;    // safe gains for going home
    private final ModelTuner model = new ModelTuner();
    private boolean plantMeasured = false;
    // translation values being tuned: {FWD, STR}
    private final double[] kEnd = new double[3];       // FWD, STR, heading
    private final double[] velGain = new double[3];

    // ---- per-run scratch ----
    private final java.util.ArrayList<double[]> samples = new java.util.ArrayList<>(); // {t, v}
    private double phaseStartX, phaseStartY, phaseStartH;
    private double rampPower = 0;
    private double lurchRefX, lurchRefY, lurchRefH;

    // search state
    private int sIdx = 0;                    // 0=X(forward), 1=Y(strafe), 2=H(turn)
    private double curTs, curZeta, curEnd;
    private int sIter;
    private boolean searching = false;
    private boolean nextAxisAfterReturn = false;
    private final double[] lockedTs = { Double.NaN, Double.NaN, Double.NaN };
    private final double[] bestCost = { Double.NaN, Double.NaN, Double.NaN };
    private enum Stage { TS, ZETA, END, CONFIRM }
    private Stage stage;
    // golden section bracket in u = ln(ts); c < d are the interior points
    private double gsA, gsB, gsC, gsD, gsFc, gsFd;
    private boolean gsPendingC;
    // ZETA / END stages: queued tries, then one parabola-vertex try
    private final java.util.ArrayDeque<double[]> tryQueue = new java.util.ArrayDeque<>();
    private double[] stageBase;
    private boolean vertexTried;
    private final java.util.ArrayList<double[]> evals = new java.util.ArrayList<>(); // {ts, zeta, end, cost}
    private double lastCost = Double.NaN;
    // running cost of the current run
    private double costInt, costLastT, costLastF;

    // active path run
    private static class Item {
        final double[][] rel; final double hdgDelta;
        Item(double[][] rel, double hdgDelta) { this.rel = rel; this.hdgDelta = hdgDelta; }
    }
    private final Item[] PROBE = {
            new Item(new double[][]{{100, 0}}, 0),    // field X = forward
            new Item(new double[][]{{0, 100}}, 0),    // field Y = strafe
            new Item(new double[][]{{0, 80}}, 45)     // heading
    };
    private final Item[] BATTERY = {
            new Item(new double[][]{{0, 100}}, 0),
            new Item(new double[][]{{0, -100}}, 0),
            new Item(new double[][]{{100, 0}}, 0),
            new Item(new double[][]{{-100, 0}}, 0),
            new Item(new double[][]{{70, 70}}, 0),
            new Item(new double[][]{{50, 50}, {100, 0}}, 0),
            new Item(new double[][]{{0, 80}}, 45)
    };
    private int batteryIdx = 0;
    private double startX, startY, startHeading;
    private double targetX, targetY, dirX, dirY, targetHeading, hdgDeltaSign;
    private TrialResult tr = new TrialResult();
    private int settleStreak = 0;
    private double settleEnterT = 0;
    private int overshootStreak = 0;
    private boolean sustainedOvershoot = false;
    // cost metrics for the probed axis
    private double tCoarse;
    private double settledAt;
    private boolean wasInZone;
    private int fineExits, reversals, lastVelSign;

    private final com.qualcomm.robotcore.util.ElapsedTime phaseTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();
    private double lastSampleT = 0;
    private int plateauCount = 0;
    private double lastV = 0;
    private FileWriter pathCsv;

    // ============================= LIFECYCLE =============================
    @Override
    public void initEX() {
        odometry.startPosition((int) CX, (int) CY, 0);
        driveBase.speed = 1;

        PathingValues.ensureLoaded();
        kEnd[0] = PathingValues.K_END_FWD;
        kEnd[1] = PathingValues.K_END_STR;
        velGain[0] = PathingValues.VEL_GAIN_FWD;
        velGain[1] = PathingValues.VEL_GAIN_STR;
        kEnd[2] = PathingValues.K_END_H;
        velGain[2] = PathingValues.VEL_GAIN_H;
        rebuildFollowers();

        telemetry.addLine("PathTuner (self tuning). Robot at CENTRE, facing 0.");
        telemetry.addLine("No values to set - everything is measured.");
        telemetry.addLine("dpad_up = START | B = STOP + SAVE | X = SKIP STEP");
        telemetry.addLine("Y = TOGGLE OUTPUT MODE (as tested / full power)");
        telemetry.addLine("dpad L/R = test power | bumpers = area size | A = accuracy preset");
    }

    @Override
    public void loopEX() {
        boolean start = currentGamepad1.dpad_up && !lastGamepad1.dpad_up;
        boolean stop  = currentGamepad1.b && !lastGamepad1.b;
        boolean skip  = currentGamepad1.x && !lastGamepad1.x;
        boolean toggleMode = currentGamepad1.y && !lastGamepad1.y;

        long now = System.nanoTime();
        if (lastLoopNanos != 0) {
            double dt = (now - lastLoopNanos) / 1e9;
            if (dt > 0.002 && dt < 0.2) loopDt += 0.05 * (dt - loopDt);
        }
        lastLoopNanos = now;

        if (toggleMode) {
            fullPowerOutput = !fullPowerOutput;
            // already done? re-save straight away in the newly selected mode
            if (state == State.FINISHED && plantMeasured) saveBest();
        }

        if (stop && state != State.IDLE && state != State.FINISHED) {
            if (searching) restoreBeforeSave();
            if (plantMeasured) saveBest();
            stopDrive();
            state = State.FINISHED;
        }

        odometry.queueCommand(odometry.update);

        switch (state) {
            case IDLE:
                stopDrive();
                adjustLimits();
                if (start) {
                    rebuildFollowers();
                    openLog();
                    beginNoise();
                }
                break;
            case NOISE:        runNoise(skip);      break;
            case BREAK_RAMP:   runBreakRamp(skip);  break;
            case BREAK_LURCH:  runBreakLurch();     break;
            case BREAK_RETURN:
                if (returnHome()) {
                    axIdx++;
                    if (axIdx >= 3) { finishBreakaway(); beginMaxVa(0); }
                    else beginBreak(axIdx);
                }
                break;
            case MAXVA_PREP:
                if (returnHome() || skip) {
                    setHome(CX, CY);
                    startMaxVaPush();
                }
                break;
            case MAXVA_PUSH:   runMaxVaPush(skip);  break;
            case MAXVA_BRAKE:  runMaxVaBrake();     break;
            case MAXVA_RETURN:
                if (returnHome()) {
                    axIdx++;
                    if (axIdx >= 3) { finishMaxVa(); beginSearchAxis(0); }
                    else beginMaxVa(axIdx);
                }
                break;
            case SEARCH_PROBE: runPath(skip);       break;
            case SEARCH_RETURN:
                if (returnHome()) {
                    if (nextAxisAfterReturn) {
                        nextAxisAfterReturn = false;
                        beginSearchAxis(sIdx + 1);
                    } else {
                        searchDecision();
                    }
                }
                break;
            case VERIFY_RUN:   runPath(skip);       break;
            case VERIFY_RETURN:
                if (returnHome()) {
                    batteryIdx++;
                    if (batteryIdx >= BATTERY.length) state = State.FINISHED;
                    else {
                        PathLog.note("VERIFY " + (batteryIdx + 1));
                        beginPath(BATTERY[batteryIdx]);
                        state = State.VERIFY_RUN;
                    }
                }
                break;
            case FINISHED:
                stopDrive();
                break;
        }

        drawTelemetry();
    }

    // ============================= 1. NOISE =============================
    private void beginNoise() {
        markPhaseStart();
        posNoise = 0; hdgNoise = 0; velNoiseT = 0; velNoiseH = 0;
        state = State.NOISE;
    }

    private void runNoise(boolean skip) {
        stopDrive();
        double dp = Math.hypot(odometry.X() - phaseStartX, odometry.Y() - phaseStartY);
        double dh = Math.abs(wrap180(odometry.Heading() - phaseStartH));
        double vt = Math.hypot(odometry.getXVelocity(), odometry.getYVelocity());
        double vh = Math.abs(degPerSec());
        if (dp > posNoise) posNoise = dp;
        if (dh > hdgNoise) hdgNoise = dh;
        if (vt > velNoiseT) velNoiseT = vt;
        if (vh > velNoiseH) velNoiseH = vh;

        if (phaseTimer.seconds() >= NOISE_TIME || skip) {
            // movement detect = clearly above standstill noise
            moveThreshT = Math.max(0.6, posNoise * 4 + 0.3);
            moveThreshH = Math.max(1.0, hdgNoise * 4 + 0.5);
            // "stopped" = velocity indistinguishable from standstill
            stopEpsT = Math.max(2.0, velNoiseT * 3 + 1.0);
            stopEpsH = Math.max(3.0, velNoiseH * 3 + 1.5);
            beginBreak(0);
        }
    }

    // ============================= 2. BREAKAWAY =============================
    private void beginBreak(int i) {
        axIdx = i;
        markPhaseStart();
        rampPower = 0;
        state = State.BREAK_RAMP;
    }

    private void runBreakRamp(boolean skip) {
        double rampCap = Math.min(RAMP_MAX_POWER, maxPower);
        rampPower = Math.min(rampCap, phaseTimer.seconds() * RAMP_RATE);
        applyAxisPower(Ax.values()[axIdx], rampPower);

        // detect on VELOCITY first (fires with far less latency than waiting
        // for displacement to accumulate, so kS isn't overestimated) with the
        // displacement check kept as a backstop
        boolean moved;
        if (Ax.values()[axIdx] == Ax.TURN) {
            moved = Math.abs(degPerSec()) > stopEpsH * 1.5
                    || Math.abs(wrap180(odometry.Heading() - phaseStartH)) > moveThreshH;
        } else {
            moved = Math.hypot(odometry.getXVelocity(), odometry.getYVelocity()) > stopEpsT * 1.5
                    || Math.hypot(odometry.X() - phaseStartX, odometry.Y() - phaseStartY) > moveThreshT;
        }

        if (moved || rampPower >= rampCap || skip) {
            ks[axIdx] = moved ? rampPower : currentKs(axIdx);   // skipped / never moved: keep the known value
            stopDrive();
            lurchRefX = phaseStartX; lurchRefY = phaseStartY; lurchRefH = phaseStartH;
            phaseTimer.reset();
            state = State.BREAK_LURCH;
        }
    }

    /** Watch how far the breakaway lurch actually travels - that distance is
     *  the smallest move this drivetrain can reliably make, so it (with margin)
     *  becomes the stop tolerance. */
    private void runBreakLurch() {
        stopDrive();
        if (phaseTimer.seconds() >= LURCH_TIME) {
            if (Ax.values()[axIdx] == Ax.TURN) {
                quantum[axIdx] = Math.abs(wrap180(odometry.Heading() - lurchRefH));
            } else {
                quantum[axIdx] = Math.hypot(odometry.X() - lurchRefX, odometry.Y() - lurchRefY);
            }
            buildReturnPath();
            state = State.BREAK_RETURN;
            phaseTimer.reset();
        }
    }

    private void finishBreakaway() {
        model.ksFwd  = ks[Ax.FWD.ordinal()];
        model.ksStr  = ks[Ax.STR.ordinal()];
        model.ksTurn = ks[Ax.TURN.ordinal()];
        // tolerance = 1.5x the lurch quantum, floored at odometry noise x4 and
        // CEILINGED - a noisy lurch measurement must not inflate tolerance so
        // far that the endpoint Kp cap (kS/tol) collapses the gains to nothing
        // the preset's tolerance, but never below the smallest move the drive
        // can reliably make (1.2 x the breakaway lurch) - tighter would hunt
        model.tolX = Math.max(PRESETS[preset][0], Math.min(5.0, Math.max(1.2 * quantum[Ax.FWD.ordinal()], posNoise * 4)));
        model.tolY = Math.max(PRESETS[preset][0], Math.min(5.0, Math.max(1.2 * quantum[Ax.STR.ordinal()], posNoise * 4)));
        model.tolH = Math.max(PRESETS[preset][1], Math.min(5.0, Math.max(1.2 * quantum[Ax.TURN.ordinal()], hdgNoise * 4)));
        PathingValues.KS_FWD = model.ksFwd;
        PathingValues.KS_STR = model.ksStr;
        PathingValues.KS_TURN = model.ksTurn;
    }

    // ============================= 3. MAX V / A =============================
    /** Forward / strafe pushes start near the far edge (twice the runway of
     *  the centre) so the speed actually levels off - a push cut short left
     *  the response time and speed-per-power poorly determined. */
    private void beginMaxVa(int i) {
        axIdx = i;
        double in = areaSize / 2 - PUSH_START_IN;
        if (Ax.values()[i] == Ax.FWD) setHome(CX - in, CY);
        else if (Ax.values()[i] == Ax.STR) setHome(CX, CY - in);
        else { startMaxVaPush(); return; }
        buildReturnPath();
        phaseTimer.reset();
        state = State.MAXVA_PREP;
    }

    private void startMaxVaPush() {
        markPhaseStart();
        samples.clear();
        posSamples.clear();
        pushOnT = Double.NaN;
        pushLastH = odometry.Heading();
        pushTurned = 0;
        lastSampleT = 0;
        plateauCount = 0;
        lastV = 0;
        modelSamples.clear();
        state = State.MAXVA_PUSH;
    }

    private void runMaxVaPush(boolean skip) {
        double t = phaseTimer.seconds();
        // stop pushing while there is still room to stop on BRAKE alone
        boolean boxHit = false;
        if (Ax.values()[axIdx] != Ax.TURN) {
            double v = Math.hypot(odometry.getXVelocity(), odometry.getYVelocity());
            double decel = brakeDecel[axIdx] > 1 ? brakeDecel[axIdx] : BRAKE_GUESS;
            boxHit = outsideBox(Math.min(areaSize / 2 - 5, v * v / (2 * decel) + 8));
        }
        boolean timeUp = t >= MAXVA_MAX_TIME;

        if (boxHit || timeUp || skip) { finishMaxVaRun(false); return; }

        applyAxisPower(Ax.values()[axIdx], maxPower);
        recordModelSample(t);
        if (Double.isNaN(pushOnT)) pushOnT = t;
        pushTurned += wrap180(odometry.Heading() - pushLastH);
        pushLastH = odometry.Heading();

        if (t - lastSampleT < SAMPLE_DT_MIN) return;
        double v = (Ax.values()[axIdx] == Ax.TURN)
                ? Math.abs(degPerSec())
                : Math.hypot(odometry.getXVelocity(), odometry.getYVelocity());
        if (samples.size() < MAX_SAMPLES) {
            samples.add(new double[]{t, v, degPerSec()});
            double d = (Ax.values()[axIdx] == Ax.TURN) ? Math.abs(pushTurned)
                    : Math.hypot(odometry.X() - phaseStartX, odometry.Y() - phaseStartY);
            posSamples.add(new double[]{t, d});
        }
        lastSampleT = t;

        if (lastV > 1) {
            plateauCount = (Math.abs(v - lastV) / lastV < PLATEAU_TOL) ? plateauCount + 1 : 0;
        }
        lastV = v;
        if (plateauCount >= PLATEAU_N) finishMaxVaRun(true);
    }

    private void finishMaxVaRun(boolean plateau) {
        plateauOk[axIdx] = plateau;
        double vBest = 0;
        for (double[] s : samples) if (s[1] > vBest) vBest = s[1];
        maxV[axIdx] = vBest;
        maxA[axIdx] = rampSlope(vBest) / maxPower;   // least-squares, normalised to per unit power
        if (axIdx < 2) {
            // mean yaw rate once up to speed
            double sum = 0;
            int n = 0;
            for (double[] s : samples) if (s[1] >= 0.6 * vBest) { sum += s[2]; n++; }
            pushYawOk[axIdx] = n >= 4 && vBest > 20;
            pushYaw[axIdx] = n > 0 ? sum / n : 0;
        }
        fitStep(axIdx);
        brakeSamples.clear();
        brakeV0 = 0;
        brakeCoast = false;
        lastSampleT = 0;
        phaseTimer.reset();
        state = State.MAXVA_BRAKE;
    }

    /** First order + dead time fitted to the push displacement:
     *  x(t) = V*g(t),  g = u - tau*(1 - e^(-u/tau)),  u = t - powerOn - T > 0.
     *  Grid over T and tau, V by least squares. Position (not the velocity
     *  estimate) so the odometry velocity filter does not pose as delay. */
    private void fitStep(int i) {
        stepV[i] = 0;
        if (posSamples.size() < 8 || Double.isNaN(pushOnT)) return;
        double best = Double.MAX_VALUE;
        for (int a = 0; a <= 60; a++) {
            double T = a * 0.0025;
            for (int b = 1; b <= 60; b++) {
                double tau = b * 0.01;
                double sgg = 0, sxg = 0, sxx = 0;
                for (double[] p : posSamples) {
                    double u = p[0] - pushOnT - T;
                    double g = u <= 0 ? 0 : u - tau * (1 - Math.exp(-u / tau));
                    sgg += g * g;
                    sxg += p[1] * g;
                    sxx += p[1] * p[1];
                }
                if (sgg <= 0) continue;
                double V = sxg / sgg;
                double err = sxx - V * sxg;
                if (err < best) { best = err; stepDelay[i] = T; stepTau[i] = tau; stepV[i] = V; }
            }
        }
    }

    /** Least-squares slope of v(t) over the ramp portion (10%..70% of peak).
     *  Immune to single-sample odometry noise, unlike a peak dV/dt. */
    private double rampSlope(double vPeak) {
        return slope(samples, 0.10 * vPeak, 0.70 * vPeak);
    }

    private static double slope(java.util.List<double[]> pts, double lo, double hi) {
        double n = 0, st = 0, sv = 0, stt = 0, stv = 0;
        for (double[] s : pts) {
            if (s[1] < lo || s[1] > hi) continue;
            n++; st += s[0]; sv += s[1]; stt += s[0] * s[0]; stv += s[0] * s[1];
        }
        if (n < 4) return 0;                       // not enough ramp data
        double denom = n * stt - st * st;
        if (Math.abs(denom) < 1e-9) return 0;
        return (n * stv - st * sv) / denom;        // cm/s^2 (or deg/s^2) at power 1.0
    }

    /** Full reverse through DriveBase's braking-current ramp (no shock on the
     *  bare motors), then 0 power once nearly stopped so it does not drive
     *  backwards. The decel measured here is what the path profile may demand. */
    private void runMaxVaBrake() {
        double t = phaseTimer.seconds();
        double v = (Ax.values()[axIdx] == Ax.TURN)
                ? Math.abs(degPerSec())
                : Math.hypot(odometry.getXVelocity(), odometry.getYVelocity());
        if (v > brakeV0) brakeV0 = v;
        if (v < 0.15 * brakeV0) brakeCoast = true;
        if (brakeCoast) stopDrive();
        else applyAxisPower(Ax.values()[axIdx], -maxPower);
        recordModelSample(t + MAXVA_MAX_TIME + 1);
        double eps = (Ax.values()[axIdx] == Ax.TURN) ? stopEpsH : stopEpsT;
        if (t - lastSampleT >= SAMPLE_DT_MIN && brakeSamples.size() < MAX_SAMPLES) {
            brakeSamples.add(new double[]{t, v});
            lastSampleT = t;
        }
        if (v < eps || t > BRAKE_MAX_TIME) {
            double v0 = 0;
            for (double[] s : brakeSamples) if (s[1] > v0) v0 = s[1];
            double d = -slope(brakeSamples, 0.15 * v0, 0.85 * v0);
            if (!(d > 1)) d = chordDecel(brakeSamples, v0);
            if (d > 1) brakeDecel[axIdx] = d;
            fitModel(axIdx);
            buildReturnPath();
            state = State.MAXVA_RETURN;
            phaseTimer.reset();
        }
    }

    private void recordModelSample(double t) {
        if (modelSamples.size() >= 4 * MAX_SAMPLES) return;
        double[] ap = driveBase.appliedAxisPowers();
        double v;
        if (Ax.values()[axIdx] == Ax.TURN) {
            v = degPerSec();
        } else {
            double h = Math.toRadians(odometry.Heading());
            double vx = odometry.getXVelocity(), vy = odometry.getYVelocity();
            v = Ax.values()[axIdx] == Ax.FWD ? vy * Math.sin(h) + vx * Math.cos(h) : vy * Math.cos(h) - vx * Math.sin(h);
        }
        modelSamples.add(new double[]{t, ap[axIdx], v});
    }

    /** Fits  dv/dt = a*p(t - n loops) + b*v + c*sign(v)  to the push + brake
     *  (the power the wheels really got, so the braking ramp is included).
     *  tau = -1/b, speed per power = a*tau. Two very different power levels
     *  make this well determined, unlike a step fit on a push that never
     *  levelled off (on the robot: forward 0.46 s from the step fit, 0.33 s
     *  from this regression on its logs). */
    private void fitModel(int axis) {
        fitOk[axis] = false;
        int n = modelSamples.size();
        if (n < 20) return;
        double[] t = new double[n], p = new double[n], v = new double[n];
        for (int i = 0; i < n; i++) {
            t[i] = modelSamples.get(i)[0];
            p[i] = modelSamples.get(i)[1];
            v[i] = modelSamples.get(i)[2];
        }
        double[] vs = new double[n];
        for (int i = 0; i < n; i++) {
            double s = 0;
            int c = 0;
            for (int k = Math.max(0, i - 1); k <= Math.min(n - 1, i + 1); k++) { s += v[k]; c++; }
            vs[i] = s / c;
        }
        double vMin = axis == Ax.TURN.ordinal() ? 15 : 8;
        double bestR2 = -1;
        for (int lag = 0; lag <= 5; lag++) {
            double[][] A = new double[3][3];
            double[] B = new double[3];
            double sy = 0, syy = 0;
            int m = 0;
            for (int i = lag + 2; i < n - 2; i++) {
                double dtc = t[i + 2] - t[i - 2];
                if (dtc <= 0 || dtc > 0.2) continue;   // skips the push / brake seam
                if (t[i] - t[i - lag] > 0.05 * (lag + 1)) continue;
                if (Math.abs(vs[i]) < vMin) continue;
                double y = (vs[i + 2] - vs[i - 2]) / dtc;
                double[] x = {p[i - lag], vs[i], Math.signum(vs[i])};
                for (int r = 0; r < 3; r++) {
                    B[r] += x[r] * y;
                    for (int c = 0; c < 3; c++) A[r][c] += x[r] * x[c];
                }
                sy += y;
                syy += y * y;
                m++;
            }
            if (m < 15) continue;
            double[] c = solve3(A, B);
            if (c == null) continue;
            double sse = syy - (c[0] * B[0] + c[1] * B[1] + c[2] * B[2]);
            double sst = syy - sy * sy / m;
            double r2 = sst > 0 ? 1 - sse / sst : 0;
            if (r2 > bestR2 && c[1] < 0) {
                bestR2 = r2;
                fitTau[axis] = -1 / c[1];
                fitSpp[axis] = c[0] * fitTau[axis];
            }
        }
        fitR2[axis] = bestR2;
        double sppMeasured = maxV[axis] / Math.max(0.1, maxPower - ks[axis]);
        fitOk[axis] = bestR2 >= MODEL_R2_MIN && fitTau[axis] >= TAU_MIN && fitTau[axis] <= TAU_MAX
                && fitSpp[axis] > 0.5 * sppMeasured && fitSpp[axis] < 2.0 * sppMeasured;
    }

    private static double[] solve3(double[][] A, double[] b) {
        double det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
                - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
                + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
        if (Math.abs(det) < 1e-12) return null;
        double[] x = new double[3];
        for (int k = 0; k < 3; k++) {
            double[][] M = new double[3][3];
            for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) M[r][c] = c == k ? b[r] : A[r][c];
            x[k] = (M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
                    - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
                    + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0])) / det;
        }
        return x;
    }

    /** Average decel between the 90 % and 10 % speed points - used when there
     *  are too few samples for the regression. */
    private static double chordDecel(java.util.List<double[]> pts, double v0) {
        double[] a = null, b = null;
        for (double[] s : pts) {
            if (a == null && s[1] <= 0.9 * v0) a = s;
            if (a != null && s[1] >= 0.1 * v0) b = s;
        }
        if (a == null || b == null || b[0] - a[0] < 0.02) return 0;
        return (a[1] - b[1]) / (b[0] - a[0]);
    }

    /** Speed per unit power above breakaway; the fitted asymptote is used when
     *  the push hit the box before the speed plateaued. */
    private double speedPerPower(int i) {
        if (!plateauOk[i] && fitOk[i]) return fitSpp[i];
        double v = maxV[i];
        if (!plateauOk[i] && stepV[i] > v) v = Math.min(stepV[i], 1.3 * maxV[i]);
        double net = maxPower - ks[i];
        return v / (net > 0.1 ? net : maxPower);
    }

    private double responseTime(int i, double current) {
        if (fitOk[i]) return clamp(fitTau[i], TAU_MIN, TAU_MAX);
        if (stepV[i] > 1) return clamp(stepTau[i], TAU_MIN, TAU_MAX);
        return current;
    }

    /** Writes the measured drive model into PathingValues - these are
     *  measurements, so they are global from here on. */
    private void finishMaxVa() {
        applyBounds();

        int f = Ax.FWD.ordinal(), st = Ax.STR.ordinal(), tu = Ax.TURN.ordinal();
        if (maxV[f] > 1) PathingValues.SPEED_PER_POWER_FWD = speedPerPower(f);
        if (maxV[st] > 1) PathingValues.SPEED_PER_POWER_STR = speedPerPower(st);
        if (maxV[tu] > 1) PathingValues.SPEED_PER_POWER_TURN = speedPerPower(tu);
        // response time: the push+brake regression when it fits well, else the step fit
        PathingValues.TAU_FWD = responseTime(f, PathingValues.TAU_FWD);
        PathingValues.TAU_STR = responseTime(st, PathingValues.TAU_STR);
        PathingValues.TAU_TURN = responseTime(tu, PathingValues.TAU_TURN);
        double dSum = 0;
        int dN = 0;
        for (int i : new int[]{f, st}) if (stepV[i] > 1) { dSum += stepDelay[i]; dN++; }
        if (dN > 0) PathingValues.DELAY = dSum / dN;
        if (stepV[tu] > 1) PathingValues.DELAY_TURN = stepDelay[tu];
        if (brakeDecel[f] > 1) PathingValues.BRAKE_DECEL_FWD = brakeDecel[f];
        if (brakeDecel[st] > 1) PathingValues.BRAKE_DECEL_STR = brakeDecel[st];

        if (brakeDecel[tu] > 1) PathingValues.BRAKE_DECEL_TURN = brakeDecel[tu];
        // Side balance: an imbalance d acts like a turn command d * power,
        // i.e. a yaw rate of d * power * (turn speed per power). Additive on
        // whatever trim was active during the push, so repeated tunes refine it.
        double turnSpp = PathingValues.SPEED_PER_POWER_TURN;
        if (maxV[tu] > 1 && turnSpp > 1) {
            double m = PathingValues.MAX_SIDE_TRIM;
            if (pushYawOk[f]) {
                trimLrMeasured = pushYaw[f] / (turnSpp * maxPower);
                PathingValues.SIDE_TRIM_LR = clamp(PathingValues.SIDE_TRIM_LR + trimLrMeasured, -m, m);
            }
            if (pushYawOk[st]) {
                trimFbMeasured = pushYaw[st] / (turnSpp * maxPower);
                PathingValues.SIDE_TRIM_FB = clamp(PathingValues.SIDE_TRIM_FB + trimFbMeasured, -m, m);
            }
        }
        plantMeasured = true;
        rebuildFollowers();
    }

    /** Clamp measured max speed / accel into [MIN, MAX]. The bounds are
     *  FULL-POWER equivalents, so each measurement is converted to full power,
     *  clamped, then converted back to the test power. Failed measurements
     *  (<= 1) are left alone, so the existing PathingValues are kept instead
     *  of being "rescued" up to the minimum. */
    private void applyBounds() {
        for (int i = 0; i < 3; i++) {
            boolean turn = (i == Ax.TURN.ordinal());
            double vLo = turn ? MIN_V_H : MIN_V_T, vHi = turn ? MAX_V_H : MAX_V_T;
            double aLo = turn ? MIN_A_H : MIN_A_T, aHi = turn ? MAX_A_H : MAX_A_T;
            double s = fullScale(i);
            vClamped[i] = false;
            aClamped[i] = false;

            if (maxV[i] > 1) {
                double full = maxV[i] * s;
                double b = clamp(full, vLo, vHi);
                if (b != full) { vClamped[i] = true; maxV[i] = b / s; }
            }
            if (maxA[i] > 1) {
                double k = maxPower * s;             // per-unit-power -> full-power accel
                double full = maxA[i] * k;
                double b = clamp(full, aLo, aHi);
                if (b != full) { aClamped[i] = true; maxA[i] = b / k; }
            }
        }
    }

    private static double positiveOr(double v, double fallback) {
        return (Double.isNaN(v) || v <= 1) ? fallback : v;
    }

    // ============================= FULL-POWER SCALING =============================
    /** Factor that turns a speed measured at the test power into the speed at
     *  power 1.0. Friction-aware: steady-state speed ~ (power - kS), so
     *  scale = (1 - kS) / (testPower - kS). Falls back to 1/power if kS is
     *  unknown or too close to the test power. Always >= 1. */
    private double fullScale(int axisIdx) {
        double p = maxPower;
        if (p >= 0.99) return 1.0;
        double denom = p - ks[axisIdx];
        double s = (denom < 0.1) ? 1.0 / p : (1.0 - ks[axisIdx]) / denom;
        return Math.max(1.0, s);
    }

    // ============================= 4. SEARCH =============================
    /** Per axis, minimising run cost:
     *   TS      - golden section on ln(ts) (stiffness), zeta 1, default endpoint
     *   ZETA    - two damping tries around the best, then the parabola vertex
     *   END     - same for endpoint softness (translation only)
     *   CONFIRM - rerun the best; if it was a fluke, take the runner-up
     *  Each run starts where the last ended (ping-pong), so no drive home. */
    private void beginSearchAxis(int i) {
        sIdx = i;
        axisStart = sIdx < 2 ? new double[]{kEnd[sIdx], velGain[sIdx], 0}
                : new double[]{kEnd[2], velGain[2], 0};
        sIter = 0;
        searching = true;
        evals.clear();
        stage = Stage.TS;
        double kHi = Math.min(KEND_HI, 1.25 * Follower.kEndLimit(sIdx == 2 ? PathingValues.DELAY_TURN : PathingValues.DELAY, loopDt));
        double kLo = Math.min(KEND_LO, kHi / 3);
        gsA = Math.log(kLo);
        gsB = Math.log(kHi);
        gsC = gsB - GOLD * (gsB - gsA);
        gsD = gsA + GOLD * (gsB - gsA);
        gsFc = Double.NaN;
        gsFd = Double.NaN;
        gsPendingC = true;
        runProbeAt(Math.exp(gsC), 1.0, 0);
    }

    private double[] axisStart;

    /** Stopped mid-search: put back the best value tried so far (or what the
     *  axis had before), never the candidate that happened to be running. */
    private void restoreBeforeSave() {
        double[] b = evals.isEmpty() ? null : bestEval(null);
        if (b != null) applyParams(b[0], b[1], b[2]);
        else if (axisStart != null) applyParams(axisStart[0], axisStart[1], axisStart[2]);
    }

    private double currentKs(int i) {
        return i == 0 ? PathingValues.KS_FWD : i == 1 ? PathingValues.KS_STR : PathingValues.KS_TURN;
    }

    private int invalidRuns = 0;
    private boolean invalidRun = false;

    private void searchDecision() {
        // started outside the box - position problem, not a gains verdict;
        // rerun the SAME params without spending an evaluation (bounded)
        if (invalidRun && invalidRuns < 3) {
            invalidRuns++;
            runProbeAt(curTs, curZeta, curEnd);
            return;
        }
        invalidRuns = 0;

        double cost = runCost();
        lastCost = cost;
        sIter++;

        if (stage == Stage.CONFIRM) {
            double[] best = bestEval(null);
            if (cost > best[3] * 1.4 + 0.3 && evals.size() > 1) {
                double[] second = bestEval(best);
                bestCost[sIdx] = second[3];
                lockAxis(second);
            } else {
                bestCost[sIdx] = Math.max(best[3], cost);
                lockAxis(best);
            }
            return;
        }

        evals.add(new double[]{curTs, curZeta, curEnd, cost});

        if (stage == Stage.TS) {
            if (gsPendingC) gsFc = cost; else gsFd = cost;
            if (Double.isNaN(gsFd)) {
                gsPendingC = false;
                runProbeAt(Math.exp(gsD), 1.0, 0);
                return;
            }
            if (evals.size() >= GS_EVALS) {
                startTweakStage(Stage.ZETA);
                return;
            }
            if (gsFc < gsFd) {
                gsB = gsD; gsD = gsC; gsFd = gsFc;
                gsC = gsB - GOLD * (gsB - gsA);
                gsPendingC = true;
                runProbeAt(Math.exp(gsC), 1.0, 0);
            } else {
                gsA = gsC; gsC = gsD; gsFc = gsFd;
                gsD = gsA + GOLD * (gsB - gsA);
                gsPendingC = false;
                runProbeAt(Math.exp(gsD), 1.0, 0);
            }
            return;
        }

        // ZETA / END
        if (!tryQueue.isEmpty()) {
            double[] p = tryQueue.poll();
            runProbeAt(p[0], p[1], p[2]);
            return;
        }
        if (!vertexTried) {
            vertexTried = true;
            double[] v = vertexCandidate();
            if (v != null) {
                runProbeAt(v[0], v[1], v[2]);
                return;
            }
        }
        {
            stage = Stage.CONFIRM;
            double[] b = bestEval(null);
            runProbeAt(b[0], b[1], b[2]);
        }
    }

    /** Two tries either side of the current best along one parameter. */
    private void startTweakStage(Stage s) {
        stage = s;
        stageBase = bestEval(null);
        vertexTried = false;
        tryQueue.clear();
        double[] tries = VG_TRY;
        for (double x : tries) {
            double[] p = stageBase.clone();
            if (s == Stage.ZETA) p[1] = x; else p[2] = x;
            tryQueue.add(p);
        }
        double[] p = tryQueue.poll();
        runProbeAt(p[0], p[1], p[2]);
    }

    /** Vertex of the parabola through the stage's three points (base + 2 tries),
     *  or null if the fit is not a valley or lands on a point already tried. */
    private double[] vertexCandidate() {
        boolean z = stage == Stage.ZETA;
        int k = z ? 1 : 2;
        java.util.ArrayList<double[]> pts = new java.util.ArrayList<>();
        for (double[] e : evals) {
            if (e[0] != stageBase[0]) continue;
            if (z ? e[2] != stageBase[2] : e[1] != stageBase[1]) continue;
            pts.add(e);
        }
        if (pts.size() < 3) return null;
        double[] p1 = pts.get(pts.size() - 3), p2 = pts.get(pts.size() - 2), p3 = pts.get(pts.size() - 1);
        boolean lin = false;
        double x1 = px(p1[k], lin), x2 = px(p2[k], lin), x3 = px(p3[k], lin);
        double y1 = p1[3], y2 = p2[3], y3 = p3[3];
        double den = (x1 - x2) * (x1 - x3) * (x2 - x3);
        if (Math.abs(den) < 1e-9) return null;
        double a = (x3 * (y2 - y1) + x2 * (y1 - y3) + x1 * (y3 - y2)) / den;
        double b = (x3 * x3 * (y1 - y2) + x2 * x2 * (y3 - y1) + x1 * x1 * (y2 - y3)) / den;
        if (a <= 0) return null;
        double xv = -b / (2 * a);
        double lo = Math.log(VG_LO), hi = Math.log(VG_HI);
        xv = clamp(xv, lo, hi);
        double minGap = 0.08;
        if (Math.abs(xv - x1) < minGap || Math.abs(xv - x2) < minGap || Math.abs(xv - x3) < minGap) return null;
        double[] c = stageBase.clone();
        c[k] = lin ? xv : Math.exp(xv);
        return c;
    }

    /** Search coordinate (log space). */
    private static double px(double v, boolean linear) { return linear ? v : Math.log(v); }

    /** Lowest-cost evaluation, skipping {@code exclude}. */
    private double[] bestEval(double[] exclude) {
        double[] best = null;
        for (double[] e : evals) {
            if (e == exclude) continue;
            if (best == null || e[3] < best[3]) best = e;
        }
        return best;
    }

    /** Lower is better: integral of the soft error penalty over COST_HORIZON
     *  (the error at the end of the run is assumed to persist to the end of the
     *  window), plus a small charge per direction reversal once in the area. */
    private double runCost() {
        if (tr.leftBox) return COST_LEFT_BOX;
        return costInt + costLastF * Math.max(0, COST_HORIZON - costLastT) + W_TWITCH * reversals
                + W_CHATTER * chatter + costOff;
    }

    private double penalty(double err) {
        return probeIsHeading() ? softPenalty(err - holdTolH(), BALANCE_H) : softPenalty(err - holdTolT(), BALANCE_T);
    }

    private static double softPenalty(double err, double balance) {
        double r = Math.max(0, err) / balance;
        return 1 - Math.exp(-r * r);
    }

    private void runProbeAt(double ts, double zeta, double end) {
        curTs = ts;
        curZeta = zeta;
        curEnd = end;
        applyParams(ts, zeta, end);
        buildTunedFollower();
        PathLog.note(sIdx < 2
                ? String.format("SEARCH %s %s kEnd=%.2f velGain=%.2f", axisName(sIdx), stage, ts, zeta)
                : String.format("SEARCH H %s ts=%.2f zeta=%.2f end=%.2f", stage, ts, zeta, end));
        beginPath(PROBE[sIdx]);
        state = State.SEARCH_PROBE;
    }

    private void lockAxis(double[] e) {
        lockedTs[sIdx] = e[0];
        applyParams(e[0], e[1], e[2]);
        // from here on, drive home with the tuned values
        rebuildFollowers();
        saveBest();
        // re-centre before the next axis / the verify battery
        buildReturnPath();
        phaseTimer.reset();
        if (sIdx >= 2) {
            searching = false;
            batteryIdx = -1;
            state = State.VERIFY_RETURN;
        } else {
            nextAxisAfterReturn = true;
            state = State.SEARCH_RETURN;
        }
    }

    /** ts slot = kEnd, zeta slot = speed / turn-rate feedback gain (end unused). */
    private void applyParams(double ts, double zeta, double end) {
        kEnd[sIdx] = ts;
        velGain[sIdx] = zeta;
    }

    private double axisTol(int i) { return i == 0 ? model.tolX : i == 1 ? model.tolY : model.tolH; }

    private void buildTunedFollower() {
        follow = new Follower(newConfig());
    }

    private void rebuildFollowers() {
        genConfig = newConfig();
        paths = new PathsManager(genConfig);
        safeFollow = new Follower(newConfig());
        buildTunedFollower();
    }

    /** Global values + what this tuner has measured / is trying. */
    private RobotConfig newConfig() {
        RobotConfig c = new RobotConfig();
        c.powerLimit = plantMeasured ? maxPower : Math.min(maxPower, 0.5);
        c.kEndFwd = kEnd[0];
        c.kEndStr = kEnd[1];
        c.velGainFwd = velGain[0];
        c.velGainStr = velGain[1];
        c.kEndH = kEnd[2];
        c.velGainH = velGain[2];
        c.holdT = holdTolT();
        c.holdH = holdTolH();
        c.overshoot = PRESETS[preset][2];
        c.overshootH = PRESETS[preset][3];
        return c;
    }

    // ============================= PATH RUN (probe + verify) =============================
    private void beginPath(Item it) {
        double cx = odometry.X(), cy = odometry.Y();
        startX = cx; startY = cy;
        startHeading = wrap360(odometry.Heading());

        // Aim the path TOWARD the field centre on each axis. Runs always
        // pushing the same direction compounded any return-home shortfall
        // until the robot walked itself out of the safety box.
        double sx = (cx > CX) ? -1 : 1;
        double sy = (cy > CY) ? -1 : 1;
        double[][] rel = new double[it.rel.length][2];
        for (int i = 0; i < it.rel.length; i++) {
            rel[i][0] = it.rel[i][0] * sx * pathScale();
            rel[i][1] = it.rel[i][1] * sy * pathScale();
        }

        double[] last = rel[rel.length - 1];
        targetX = cx + last[0];
        targetY = cy + last[1];
        double len = Math.hypot(last[0], last[1]);
        dirX = len > 0 ? last[0] / len : 0;
        dirY = len > 0 ? last[1] / len : 0;
        // turn back toward 0 when already turned, so chained turn probes ping-pong
        double hd = it.hdgDelta;
        if (hd != 0 && wrap180(startHeading) > 1) hd = -Math.abs(hd);
        targetHeading = wrap360(startHeading + hd);
        hdgDeltaSign = Math.signum(hd);

        final Vector2D[] pts = new Vector2D[rel.length + 1];
        pts[0] = new Vector2D(cx, cy);
        for (int i = 0; i < rel.length; i++) {
            pts[i + 1] = new Vector2D(cx + rel[i][0], cy + rel[i][1]);
        }
        SectionBuilder[] section = new SectionBuilder[]{ () -> addPts(pts) };
        paths.addNewPath("outPath");
        paths.buildPath(section);

        follow.setPath(paths.returnPath("outPath"));
        follow.usePathHeadings(false);
        follow.holdPositionAtPathEnd(true);

        tr = new TrialResult();
        tr.settleTime = SETTLE_TIMEOUT;
        invalidRun = false;
        settleStreak = 0;
        overshootStreak = 0;
        sustainedOvershoot = false;
        tCoarse = Double.NaN;
        settledAt = Double.NaN;
        wasInZone = false;
        fineExits = 0;
        reversals = 0;
        lastVelSign = 0;
        chatter = 0;
        chatterLast = Double.NaN;
        costOff = 0;
        lastOffT = 0;
        costInt = 0;
        costLastT = 0;
        costLastF = penalty(probeErr());
        phaseTimer.reset();
    }

    /** Error on the axis being judged (probed axis while searching, distance to end while verifying). */
    private double probeErr() {
        if (searching) {
            if (sIdx == 0) return Math.abs(odometry.X() - targetX);
            if (sIdx == 1) return Math.abs(odometry.Y() - targetY);
            return Math.abs(wrap180(targetHeading - odometry.Heading()));
        }
        return Math.hypot(odometry.X() - targetX, odometry.Y() - targetY);
    }

    private boolean probeIsHeading() { return searching && sIdx == 2; }

    private double probeVel() {
        if (searching) {
            if (sIdx == 0) return odometry.getXVelocity();
            if (sIdx == 1) return odometry.getYVelocity();
            return degPerSec();
        }
        return odometry.getXVelocity() * dirX + odometry.getYVelocity() * dirY;
    }

    /** "In the area" for ending a search run: a bit beyond the tolerance. */
    private double balance() { return probeIsHeading() ? Math.max(BALANCE_H, holdTolH() + 2) : Math.max(BALANCE_T, holdTolT() + 2); }

    /** Integrates the error penalty, and counts direction reversals once in the
     *  area (overshoot / hunting / twitch). */
    private void trackCost(double t) {
        boolean hdg = probeIsHeading();
        double err = probeErr();
        double f = penalty(err);
        costInt += 0.5 * (f + costLastF) * (t - costLastT);
        costLastT = t;
        costLastF = f;
        if (Double.isNaN(tCoarse) && err <= balance()) tCoarse = t;
        double v = probeVel();
        double thr = hdg ? stopEpsH * 2 : stopEpsT * 2;
        int sgn = Math.abs(v) > thr ? (v > 0 ? 1 : -1) : 0;
        if (sgn != 0) {
            if (!Double.isNaN(tCoarse) && lastVelSign != 0 && sgn != lastVelSign) reversals++;
            lastVelSign = sgn;
        }
        // the shudder lived on the axis NOT being probed, so score the whole
        // robot: chatter on every command, plus sideways / heading error
        if (!Double.isNaN(tCoarse) && !Double.isNaN(chatterLast)) {
            chatter += Math.abs(cmdF - lastCmdF) + Math.abs(cmdS - lastCmdS) + Math.abs(cmdT - lastCmdT);
        }
        chatterLast = 0;
        lastCmdF = cmdF;
        lastCmdS = cmdS;
        lastCmdT = cmdT;
        double rx = odometry.X() - startX, ry = odometry.Y() - startY;
        double side = Math.abs(rx * dirY - ry * dirX);
        double offPen = softPenalty(side - holdTolT(), BALANCE_T);
        if (!hdg) offPen += softPenalty(Math.abs(wrap180(targetHeading - odometry.Heading())) - holdTolH(), BALANCE_H);
        costOff += W_OFF_AXIS * offPen * Math.max(0, t - lastOffT);
        lastOffT = t;
    }

    private void runPath(boolean skip) {
        if (outsideBox(0)) {
            // devCount == 0 -> we were ALREADY outside before driving at all;
            // that's a bad start position, not the gains' fault
            if (tr.devCount == 0) invalidRun = true;
            tr.leftBox = true; captureFinal(); finishPath(); return;
        }
        if (skip) { tr.timedOut = true; captureFinal(); finishPath(); return; }

        drive(follow, targetHeading);
        sampleMetrics();
        double t = phaseTimer.seconds();
        trackCost(t);

        // While searching, a run ends once the robot is in the area and slow -
        // whether it then reaches a perfect settle is priced by the cost
        // integral, not by a hard tolerance. Verify runs still demand the
        // fine tolerance on every axis.
        double aX = Math.abs(odometry.X() - targetX);
        double aY = Math.abs(odometry.Y() - targetY);
        double aH = Math.abs(wrap180(targetHeading - odometry.Heading()));
        boolean inZone;
        if (searching) {
            double fx = (sIdx == 0) ? 1 : 3, fy = (sIdx == 1) ? 1 : 3, fh = (sIdx == 2) ? 1 : 3;
            inZone = aX <= model.tolX * fx && aY <= model.tolY * fy && aH <= model.tolH * fh;
        } else {
            inZone = aX <= model.tolX && aY <= model.tolY && aH <= model.tolH;
        }
        boolean inArea = searching ? probeErr() <= balance() : inZone;
        // "slow", not perfectly stopped - holdPositionAtPathEnd keeps making
        // micro-corrections, so a hard stop threshold never latches
        double vt = Math.hypot(odometry.getXVelocity(), odometry.getYVelocity());
        boolean stopped = vt < stopEpsT * 3 && Math.abs(degPerSec()) < stopEpsH * 3;

        if (!Double.isNaN(settledAt)) {
            // settled - keep holding and watch for the endpoint twitch
            if (wasInZone && !inZone) fineExits++;
            wasInZone = inZone;
            if (t >= settledAt + HOLD_WATCH) {
                captureFinal();
                finishPath();
            }
            return;
        }

        if (inArea && stopped) {
            if (settleStreak == 0) settleEnterT = t;
            settleStreak++;
            if (settleStreak >= SETTLE_N) {
                tr.settleTime = settleEnterT;
                settledAt = settleEnterT;
                wasInZone = inZone;
                return;
            }
        } else {
            settleStreak = 0;
        }

        if (t > SETTLE_TIMEOUT) {
            tr.timedOut = true;
            tr.settleTime = SETTLE_TIMEOUT;
            captureFinal();
            finishPath();
        }
    }

    private void sampleMetrics() {
        double speed = Math.hypot(odometry.getXVelocity(), odometry.getYVelocity());
        if (speed > tr.peakSpeed) tr.peakSpeed = speed;

        // overshoot measured on the axis actually being probed
        double beyond, tol;
        if (searching && sIdx == 2) {
            beyond = wrap180(odometry.Heading() - targetHeading) * hdgDeltaSign;
            tol = model.tolH;
        } else {
            double px = odometry.X() - targetX;
            double py = odometry.Y() - targetY;
            beyond = px * dirX + py * dirY;
            tol = (searching && sIdx == 0) ? model.tolX : model.tolY;
        }
        if (beyond > tr.overshoot) tr.overshoot = beyond;
        if (beyond > tol) {
            overshootStreak++;
            if (overshootStreak >= OVERSHOOT_N) sustainedOvershoot = true;
        } else {
            overshootStreak = 0;
        }

        double rx = odometry.X() - startX;
        double ry = odometry.Y() - startY;
        double along = rx * dirX + ry * dirY;
        double dev = Math.hypot(rx - along * dirX, ry - along * dirY);
        tr.sumDev += dev;
        tr.devCount++;
        if (dev > tr.maxDev) tr.maxDev = dev;
    }

    private void captureFinal() {
        tr.finalErrX = Math.abs(odometry.X() - targetX);
        tr.finalErrY = Math.abs(odometry.Y() - targetY);
        tr.finalPosErr = Math.hypot(tr.finalErrX, tr.finalErrY);
        tr.finalHdgErr = Math.abs(wrap180(targetHeading - odometry.Heading()));
    }

    private void finishPath() {
        if (!searching) logPath();
        // ping-pong: a clean run that ended in a sensible spot is the next
        // run's start, so skip the drive home
        if (searching && !tr.leftBox && !tr.timedOut
                && Math.hypot(odometry.X() - CX, odometry.Y() - CY) <= 100 * pathScale() + 15
                && Math.abs(wrap180(odometry.Heading())) <= 60) {
            searchDecision();
            return;
        }
        buildReturnPath();
        state = searching ? State.SEARCH_RETURN : State.VERIFY_RETURN;
        phaseTimer.reset();
    }

    // ============================= SHARED HELPERS =============================
    private void markPhaseStart() {
        phaseStartX = odometry.X();
        phaseStartY = odometry.Y();
        phaseStartH = odometry.Heading();
        phaseTimer.reset();
    }

    private void applyAxisPower(Ax axis, double p) {
        p = clamp(p, -maxPower, maxPower);
        switch (axis) {
            case FWD:  driveBase.queueCommand(driveBase.drivePowers(p, 0, 0)); break;
            case STR:  driveBase.queueCommand(driveBase.drivePowers(0, p, 0)); break;
            case TURN: driveBase.queueCommand(driveBase.drivePowers(0, 0, p)); break;
        }
    }

    private int returnTries = 0;

    private void buildReturnPath() {
        PathLog.note("RETURN " + state);
        returnTries = 0;
        rebuildReturnPath();
    }

    private double homeX = CX, homeY = CY;

    private void setHome(double x, double y) {
        homeX = x;
        homeY = y;
    }

    private void rebuildReturnPath() {
        final Vector2D[] pts = new Vector2D[]{
                new Vector2D(odometry.X(), odometry.Y()),
                new Vector2D(homeX, homeY)
        };
        SectionBuilder[] section = new SectionBuilder[]{ () -> addPts(pts) };
        paths.addNewPath("backPath");
        paths.buildPath(section);
        safeFollow.setPath(paths.returnPath("backPath"));
        safeFollow.usePathHeadings(false);
        safeFollow.holdPositionAtPathEnd(true);
    }

    /** @return true only when ACTUALLY near home (retries the return path if a
     *  timeout leaves the robot far away - starting the next run from a bad
     *  position was what marched the robot off the field). */
    private boolean returnHome() {
        drive(safeFollow, 0);
        boolean stopped = Math.hypot(odometry.getXVelocity(), odometry.getYVelocity()) < stopEpsT;
        boolean home = safeFollow.isFinished(8, 8);
        double distHome = Math.hypot(odometry.X() - homeX, odometry.Y() - homeY);
        if (home && stopped) {
            stopDrive();
            return true;
        }
        if (phaseTimer.seconds() > RETURN_TIMEOUT) {
            if (distHome <= HOME_OK_DIST || returnTries >= RETURN_RETRIES) {
                stopDrive();
                return true;    // close enough, or out of retries - carry on
            }
            returnTries++;
            rebuildReturnPath();  // fresh path from wherever we actually are
            phaseTimer.reset();
        }
        return false;
    }

    private void drive(Follower f, double heading) {
        RobotPower cp = f.followPathAuto(heading, odometry.Heading(),
                odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity(), degPerSec());
        cmdF = clamp(cp.getVertical(), -maxPower, maxPower);
        cmdS = clamp(cp.getHorizontal(), -maxPower, maxPower);
        cmdT = clamp(cp.getPivot(), -maxPower, maxPower);
        driveBase.queueCommand(driveBase.drivePowers(cmdF, cmdS, cmdT));
    }

    private void adjustLimits() {
        if (currentGamepad1.a && !lastGamepad1.a) {
            preset = (preset + 1) % PRESETS.length;
            model.tolX = model.tolY = PRESETS[preset][0];
            model.tolH = PRESETS[preset][1];
        }
        if (currentGamepad1.dpad_right && !lastGamepad1.dpad_right) maxPower += POWER_STEP;
        if (currentGamepad1.dpad_left && !lastGamepad1.dpad_left) maxPower -= POWER_STEP;
        if (currentGamepad1.right_bumper && !lastGamepad1.right_bumper) areaSize += AREA_STEP;
        if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper) areaSize -= AREA_STEP;
        maxPower = Math.round(clamp(maxPower, POWER_MIN, 1.0) * 100) / 100.0;
        areaSize = clamp(areaSize, AREA_MIN, AREA_MAX);
    }

    /** Path distances shrink with the area so probes always stay inside it
     *  (1.0 at the default 250 cm area). */
    private double pathScale() {
        return clamp((areaSize / 2 - 25) / 100.0, 0.2, 1.0);
    }

    private void stopDrive() {
        driveBase.queueCommand(driveBase.drivePowers(0, 0, 0));
    }

    private void addPts(Vector2D[] pts) {
        if (pts.length == 2) paths.addPoints(pts[0], pts[1]);
        else if (pts.length == 3) paths.addPoints(pts[0], pts[1], pts[2]);
        else paths.addPoints(pts[0], pts[1], pts[2], pts[3]);
    }

    private boolean outsideBox(double margin) {
        double x = odometry.X(), y = odometry.Y();
        double half = areaSize / 2;
        return x < CX - half + margin || x > CX + half - margin
                || y < CY - half + margin || y > CY + half - margin;
    }

    private double holdTolH() { return model.tolH; }
    private double holdTolT() { return Math.min(model.tolX, model.tolY); }

    private double degPerSec() { return odometry.getHeadingVelocityDeg(); }

    private static double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
    private static double wrap360(double a) { a %= 360; if (a < 0) a += 360; return a; }
    private static double wrap180(double a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }

    // ============================= TELEMETRY / LOGGING =============================
    private String modeLabel() {
        return fullPowerOutput
                ? "FULL POWER (saves POWER_LIMIT 1.0)"
                : "AS TESTED (saves POWER_LIMIT " + String.format("%.2f", maxPower) + ")";
    }

    private void drawTelemetry() {
        telemetry.addLine("dpad_up=START  B=STOP+SAVE  X=SKIP STEP");
        telemetry.addLine("Y = TOGGLE OUTPUT MODE   (idle: dpad L/R = test power, bumpers = area)");
        telemetry.addData("OUTPUT MODE (press Y)", modeLabel());
        telemetry.addData("state", state);
        telemetry.addData("TEST POWER (dpad L/R)", String.format("%.2f", maxPower));
        telemetry.addData("ACCURACY (A)", String.format("%s  tol %.1f cm / %.1f deg  overshoot %.1f cm / %.1f deg",
                PRESET_NAMES[preset], PRESETS[preset][0], PRESETS[preset][1], PRESETS[preset][2], PRESETS[preset][3]));
        telemetry.addData("TUNING AREA (bumpers)", String.format("%.0f cm  (path scale %.2f)", areaSize, pathScale()));
        telemetry.addData("noise pos/hdg/velT/velH", String.format("%.2f / %.2f / %.1f / %.1f",
                posNoise, hdgNoise, velNoiseT, velNoiseH));
        telemetry.addData("breakaway F/S/T", String.format("%.2f / %.2f / %.2f",
                ks[0], ks[1], ks[2]));
        telemetry.addData("tol X/Y/H (derived)", String.format("%.1f / %.1f / %.1f",
                model.tolX, model.tolY, model.tolH));
        telemetry.addData("maxV F/S/T (at test power)", String.format("%.0f / %.0f / %.0f  plateau %b/%b/%b",
                maxV[0], maxV[1], maxV[2], plateauOk[0], plateauOk[1], plateauOk[2]));
        if (vClamped[0] || vClamped[1] || vClamped[2] || aClamped[0] || aClamped[1] || aClamped[2])
            telemetry.addData("!! BOUNDS HIT (V F/S/T, A F/S/T)", String.format("%b/%b/%b, %b/%b/%b",
                    vClamped[0], vClamped[1], vClamped[2], aClamped[0], aClamped[1], aClamped[2]));
        telemetry.addData("speed/power F/S/T", String.format("%.0f / %.0f / %.0f",
                PathingValues.SPEED_PER_POWER_FWD, PathingValues.SPEED_PER_POWER_STR, PathingValues.SPEED_PER_POWER_TURN));
        telemetry.addData("model fit tau F/S/T (R2)", String.format("%.2f (%.2f%s) / %.2f (%.2f%s) / %.2f (%.2f%s)",
                fitTau[0], fitR2[0], fitOk[0] ? "" : " x", fitTau[1], fitR2[1], fitOk[1] ? "" : " x",
                fitTau[2], fitR2[2], fitOk[2] ? "" : " x"));
        telemetry.addData("step delay/tau ms F S T", String.format("%.0f/%.0f  %.0f/%.0f  %.0f/%.0f",
                stepDelay[0] * 1000, stepTau[0] * 1000, stepDelay[1] * 1000, stepTau[1] * 1000,
                stepDelay[2] * 1000, stepTau[2] * 1000));
        telemetry.addData("brake decel F/S/T", String.format("%.0f / %.0f / %.0f   planned F/S %.0f / %.0f",
                brakeDecel[0], brakeDecel[1], brakeDecel[2],
                PathingValues.BRAKE_DECEL_FWD * PathingValues.DECEL_MARGIN, PathingValues.BRAKE_DECEL_STR * PathingValues.DECEL_MARGIN));
        telemetry.addData("side trim L/R F/B", String.format("%+.1f%% / %+.1f%%   (push yaw %.1f / %.1f deg/s)",
                PathingValues.SIDE_TRIM_LR * 100, PathingValues.SIDE_TRIM_FB * 100, pushYaw[0], pushYaw[1]));
        telemetry.addData("loop dt", String.format("%.1f ms   max wheel speed %.2f", loopDt * 1000,
                driveBase.maxWheelSpeedFraction()));
        if (state == State.SEARCH_PROBE || state == State.SEARCH_RETURN) {
            String p = String.format("kEnd %.1f  velGain %.2f", curTs, curZeta);
            telemetry.addData("search", String.format("%s %s  %s  run %d  last cost %.2f",
                    axisName(sIdx), stage, p, sIter + 1, lastCost));
        }
        if (state == State.SEARCH_PROBE)
            telemetry.addData("this run", String.format("in area @%.2fs  stopped @%.2fs  reversals %d  fine exits %d  chatter %.2f  cost so far %.2f",
                    tCoarse, settledAt, reversals, fineExits, chatter, costInt));
        if (state == State.VERIFY_RUN || state == State.VERIFY_RETURN)
            telemetry.addData("verify path", (batteryIdx + 1) + "/" + BATTERY.length);
        telemetry.addData("kEnd X/Y/H  velGain X/Y/H", String.format("%.1f / %.1f / %.1f   %.2f / %.2f / %.2f",
                kEnd[0], kEnd[1], kEnd[2], velGain[0], velGain[1], velGain[2]));
        if (follow != null)
            telemetry.addData("follower", String.format("%s  %s  heading %s  target %.0f cm/s  to go %.1f cm",
                    follow.isSettling() ? "settling" : "path", follow.isHolding() ? "HOLD" : "",
                    follow.isHoldingHeading() ? "HOLD" : "turning", follow.getTargetSpeed(), follow.getRemaining()));
        telemetry.addData("pos", String.format("%.0f,%.0f  h%.0f",
                odometry.X(), odometry.Y(), odometry.Heading()));
        telemetry.addData("best cost X/Y/H", String.format("%.2f / %.2f / %.2f",
                bestCost[0], bestCost[1], bestCost[2]));
        if (state == State.FINISHED) telemetry.addLine(">>> DONE - saved to PathingValues + tune_best.txt (" + (fullPowerOutput ? "FULL POWER" : "AS TESTED") + ") - Y switches mode + re-saves <<<");
    }

    private static String axisName(int i) { return i == 0 ? "X" : i == 1 ? "Y" : "H"; }

    private static String fmt(double[] p) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < p.length; i++) {
            if (i > 0) sb.append(", ");
            sb.append(String.format("%.5f", p[i]));
        }
        return sb.toString();
    }

    private void openLog() {
        try {
            File dir = new File("/sdcard/FIRST/datalogs");
            if (!dir.exists()) dir.mkdirs();
            pathCsv = new FileWriter(new File(dir, "tune_paths.csv"), false);
            pathCsv.write("path," + TrialResult.csvHeader() + "\n");
            pathCsv.flush();
        } catch (IOException e) {
            telemetry.addData("LOG ERR", e.getMessage());
            pathCsv = null;
        }
    }

    private void logPath() {
        if (pathCsv == null) return;
        try {
            pathCsv.write(String.format("%d,%s\n", batteryIdx + 1, tr.csv()));
            pathCsv.flush();
        } catch (IOException ignored) {}
    }

    /** Puts the tuned values into PathingValues (live for every follower made
     *  after this), saves its file, and writes a readable summary. */
    private void saveBest() {
        PathingValues.K_END_FWD = kEnd[0];
        PathingValues.K_END_STR = kEnd[1];
        PathingValues.VEL_GAIN_FWD = velGain[0];
        PathingValues.VEL_GAIN_STR = velGain[1];
        PathingValues.K_END_H = kEnd[2];
        PathingValues.VEL_GAIN_H = velGain[2];
        PathingValues.HOLD_T = holdTolT();
        PathingValues.HOLD_H = holdTolH();
        PathingValues.OVERSHOOT = PRESETS[preset][2];
        PathingValues.OVERSHOOT_H = PRESETS[preset][3];
        PathingValues.POWER_LIMIT = fullPowerOutput ? 1.0 : maxPower;
        boolean saved = PathingValues.save();
        try {
            File dir = new File("/sdcard/FIRST/datalogs");
            if (!dir.exists()) dir.mkdirs();
            FileWriter w = new FileWriter(new File(dir, "tune_best.txt"), false);
            w.write("// PathTuner (self tuning) result\n");
            w.write("// OUTPUT MODE: " + modeLabel() + "\n");
            w.write("// " + (saved ? "saved to " : "!! could NOT save ") + PathingValues.FILE
                    + " - loaded automatically by every new RobotConfig()\n");
            w.write(String.format("// odometry noise  pos=%.2fcm hdg=%.2fdeg  vel=%.1fcm/s %.1fdeg/s\n",
                    posNoise, hdgNoise, velNoiseT, velNoiseH));
            w.write(String.format("// derived tolerances  X=%.1f Y=%.1f cm  H=%.1f deg\n",
                    model.tolX, model.tolY, model.tolH));
            w.write(String.format("// measured maxV (at test power %.2f)  fwd=%.0f strafe=%.0f turn=%.0f  [plateau %b/%b/%b]\n",
                    maxPower, maxV[0], maxV[1], maxV[2], plateauOk[0], plateauOk[1], plateauOk[2]));
            w.write(String.format("// brake decel (protected reverse)  fwd=%.0f strafe=%.0f turn=%.0f\n",
                    brakeDecel[0], brakeDecel[1], brakeDecel[2]));
            w.write(String.format("// accuracy preset %s: tol %.1f cm / %.1f deg, overshoot %.1f cm / %.1f deg\n",
                    PRESET_NAMES[preset], holdTolT(), holdTolH(), PRESETS[preset][2], PRESETS[preset][3]));
            w.write(String.format("// best cost X/Y/H %.2f / %.2f / %.2f\n", bestCost[0], bestCost[1], bestCost[2]));
            w.write(String.format("// response time fit (push+brake regression) fwd %.2f s R2 %.2f%s, strafe %.2f s R2 %.2f%s, turn %.2f s R2 %.2f%s\n",
                    fitTau[0], fitR2[0], fitOk[0] ? "" : " (rejected)", fitTau[1], fitR2[1], fitOk[1] ? "" : " (rejected)",
                    fitTau[2], fitR2[2], fitOk[2] ? "" : " (rejected)"));
            w.write(String.format("// side trim L/R %+.1f%%  F/B %+.1f%%  (push yaw fwd %.1f deg/s, strafe %.1f deg/s; this tune added %+.1f%% / %+.1f%%)\n",
                    PathingValues.SIDE_TRIM_LR * 100, PathingValues.SIDE_TRIM_FB * 100, pushYaw[0], pushYaw[1],
                    trimLrMeasured * 100, trimFbMeasured * 100));
            w.write("// To hard-code instead (set USE_TUNED_FILE = false), paste into PathingValues:\n");
            w.write(PathingValues.asJava());
            w.close();
        } catch (IOException ignored) {}
    }
}
