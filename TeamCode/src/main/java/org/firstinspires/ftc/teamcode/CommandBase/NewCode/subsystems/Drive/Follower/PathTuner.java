package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Follower;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import dev.weaponboy.nexus_pathing.Follower.Follower;
import dev.weaponboy.nexus_pathing.Follower.FollowerPrimer;
import dev.weaponboy.nexus_pathing.PathGeneration.PathsManager;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.SectionBuilder;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.RobotConfig;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

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
 *                  noise-spike gains). Then 0 power (BRAKE) - the bare motors
 *                  are never plug-reversed - and the passive brake decel is
 *                  measured. Path accel/decel is capped to what braking alone
 *                  can do, so the follower never needs reverse power to stop.
 *                  Measured values are checked against the MIN/MAX speed and
 *                  accel bounds below (clamped if outside).
 *   4. SEARCH    - per axis, golden-section search on ln(ts) minimising a
 *                  run cost: time into a 5 cm / 5 deg zone, extra time to
 *                  settle in the fine tolerance, overshoot, and direction
 *                  reversals / zone exits (twitch) during a hold watch after
 *                  settling. 6 runs + 1 confirm per axis.
 *   5. VERIFY    - battery of paths with the locked gains, logged to CSV.
 *
 * Gains come from pole placement at zeta=1 (see ModelTuner) - Kd is always
 * re-derived from the final Kp so clamping can never produce an underdamped
 * (wobbly) pair.
 *
 * OUTPUT MODE (Y button):
 *   AS TESTED  - saved values match what the robot did at the test power.
 *   FULL POWER - tune at e.g. 0.8 power for safety, but the saved gains and
 *                maxV/maxA are scaled to what the robot does at 1.0 power
 *                (friction-aware: speed ~ (power - kS)). Can be toggled any
 *                time, including after FINISHED (re-saves automatically).
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
    private static final double SAMPLE_DT_MIN   = 0.02;

    // ---- run cost: integral of a soft error penalty over a fixed window ----
    //   f(e) = 1 - exp(-(e / BALANCE)^2)
    // ~1 per second while far away, ~0.6 at BALANCE, ~0.2 at half of it, ~0
    // when spot on. Getting into the area fast dominates the score; finishing
    // precisely still earns a little; overshoot, hunting and endpoint twitch
    // all show up as error, so no hard tolerance line decides anything.
    // Smaller BALANCE = values a precise final settle more.
    private static final double BALANCE_T = 5.0, BALANCE_H = 5.0;   // cm, deg
    private static final double COST_HORIZON = 4.4;              // s - same window for every run
    private static final double W_TWITCH = 0.08;                 // s per direction reversal once in the area
    private static final double COST_LEFT_BOX = 20;

    // ---- search per axis: stiffness (golden section), then damping and
    //      endpoint softness (2 tries + parabola vertex each), then confirm ----
    private static final double TS_LO_T = 0.18, TS_HI_T = 1.2;
    private static final double TS_LO_H = 0.20, TS_HI_H = 1.4;
    private static final int    GS_EVALS = 5;
    private static final double GOLD = 0.6180339887;
    private static final double[] ZETA_TRY = {0.7, 1.35};
    private static final double ZETA_LO = 0.55, ZETA_HI = 1.6;
    private static final double[] END_TRY = {0.9, 2.0};
    private static final double END_LO = 0.75, END_HI = 2.6;
    private static final double END_DEFAULT = 4.0 / 3.0;
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

    // known-good gains ONLY for driving home between steps (never tuned/saved).
    // Only used until the first axis is tuned; after that returns run on the
    // tuned gains. Format: P, D pairs in the order
    //   X on-path, Y on-path, X last-adjust, Y last-adjust, fast heading, slow heading
    // (field X = robot forward, field Y = robot strafe (at heading 0))
    // Cut for the faster 504 rpm drivetrain: translation /2, heading /3.
    private static final double[] SAFE_GAINS = {
            0.08,    0.004,      0.15,    0.003,      // X on-path, Y on-path
            0.025,   0.004,      0.025,   0.007,      // X last-adjust, Y last-adjust
            0.00667, 0.00033,    0.00667, 0.0010      // fast heading, slow heading
    };

    // Used ONLY when a measurement fails (robot didn't move etc).
    // Keep these inside the bounds below.
    private static final double FALLBACK_XV = 220, FALLBACK_YV = 170;
    private static final double FALLBACK_XA = 700, FALLBACK_YA = 650;

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
        MAXVA_PUSH, MAXVA_BRAKE, MAXVA_RETURN,
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
    private double[] tunedGains = null;

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
            new Item(new double[][]{{100, 0}}, 0),    // X / strafe
            new Item(new double[][]{{0, 100}}, 0),    // Y / forward
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

        genConfig = buildConfig(SAFE_GAINS, FALLBACK_XV, FALLBACK_YV, FALLBACK_XA, FALLBACK_YA);
        paths = new PathsManager(genConfig);
        safeFollow = new Follower(genConfig);

        telemetry.addLine("PathTuner (self tuning). Robot at CENTRE, facing 0.");
        telemetry.addLine("No values to set - everything is measured.");
        telemetry.addLine("dpad_up = START | B = STOP + SAVE | X = SKIP STEP");
        telemetry.addLine("Y = TOGGLE OUTPUT MODE (as tested / full power)");
        telemetry.addLine("dpad L/R = test power | bumpers = area size");
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
            if (state == State.FINISHED && tunedGains != null) saveBest();
        }

        if (stop && state != State.IDLE && state != State.FINISHED) {
            if (tunedGains != null) saveBest();
            stopDrive();
            state = State.FINISHED;
        }

        odometry.queueCommand(odometry.update);

        switch (state) {
            case IDLE:
                stopDrive();
                adjustLimits();
                if (start) {
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
                    else { beginPath(BATTERY[batteryIdx]); state = State.VERIFY_RUN; }
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
            ks[axIdx] = moved ? rampPower : rampCap * 0.5; // no-move fallback
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
        model.tolX = clamp(1.5 * quantum[Ax.FWD.ordinal()], Math.max(2.5, posNoise * 4), 5.0);
        model.tolY = clamp(1.5 * quantum[Ax.STR.ordinal()], Math.max(2.5, posNoise * 4), 5.0);
        model.tolH = clamp(1.5 * quantum[Ax.TURN.ordinal()], Math.max(2.5, hdgNoise * 4), 5.0);
    }

    // ============================= 3. MAX V / A =============================
    private void beginMaxVa(int i) {
        axIdx = i;
        markPhaseStart();
        samples.clear();
        lastSampleT = 0;
        plateauCount = 0;
        lastV = 0;
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

        if (t - lastSampleT < SAMPLE_DT_MIN) return;
        double v = (Ax.values()[axIdx] == Ax.TURN)
                ? Math.abs(degPerSec())
                : Math.hypot(odometry.getXVelocity(), odometry.getYVelocity());
        if (samples.size() < MAX_SAMPLES) samples.add(new double[]{t, v});
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
        brakeSamples.clear();
        lastSampleT = 0;
        phaseTimer.reset();
        state = State.MAXVA_BRAKE;
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

    /** 0 power (motors in BRAKE) until stopped - never plug-reverse the bare
     *  motors. The decel measured here is what the path profile may demand. */
    private void runMaxVaBrake() {
        stopDrive();
        double t = phaseTimer.seconds();
        double v = (Ax.values()[axIdx] == Ax.TURN)
                ? Math.abs(degPerSec())
                : Math.hypot(odometry.getXVelocity(), odometry.getYVelocity());
        double eps = (Ax.values()[axIdx] == Ax.TURN) ? stopEpsH : stopEpsT;
        if (t - lastSampleT >= SAMPLE_DT_MIN && brakeSamples.size() < MAX_SAMPLES) {
            brakeSamples.add(new double[]{t, v});
            lastSampleT = t;
        }
        if (v < eps || t > BRAKE_MAX_TIME) {
            double v0 = 0;
            for (double[] s : brakeSamples) if (s[1] > v0) v0 = s[1];
            brakeDecel[axIdx] = -slope(brakeSamples, 0.15 * v0, 0.85 * v0);
            buildReturnPath();
            state = State.MAXVA_RETURN;
            phaseTimer.reset();
        }
    }

    /** Accel/decel the path profile may use: what the drive can do at the test
     *  power AND what BRAKE alone can stop, whichever is smaller, with margin.
     *  The profile uses one value for both, so decel never needs plugging. */
    private double profileAccel(boolean fullPower) {
        double m = Double.MAX_VALUE;
        for (Ax a : new Ax[]{Ax.FWD, Ax.STR}) {
            int i = a.ordinal();
            double drive = maxA[i] > 1 ? maxA[i] * (fullPower ? fullScale(i) : 1) * maxPower : Double.MAX_VALUE;
            double brake = brakeDecel[i] > 1 ? brakeDecel[i] : Double.MAX_VALUE;
            m = Math.min(m, Math.min(drive, brake));
        }
        if (m == Double.MAX_VALUE) m = Math.min(FALLBACK_XA, FALLBACK_YA);
        return m * PROFILE_MARGIN;
    }

    private void finishMaxVa() {
        // keep measurements inside the sanity bounds before anything uses them
        applyBounds();

        // accel-per-power straight from the regression
        model.aFwd  = positiveOr(maxA[Ax.FWD.ordinal()],  FALLBACK_YA);
        model.aStr  = positiveOr(maxA[Ax.STR.ordinal()],  FALLBACK_XA);
        model.aTurn = positiveOr(maxA[Ax.TURN.ordinal()], 300);

        double xv = positiveOr(maxV[Ax.FWD.ordinal()], FALLBACK_XV);
        double yv = positiveOr(maxV[Ax.STR.ordinal()], FALLBACK_YV);
        double acc = profileAccel(false);

        // rebuild path generation + safe follower with real constants
        genConfig = buildConfig(SAFE_GAINS, xv, yv, acc, acc);
        paths = new PathsManager(genConfig);
        safeFollow = new Follower(genConfig);

        // let the plug limiter estimate wheel speed from real full-power speeds
        if (maxV[Ax.FWD.ordinal()] > 1)  driveBase.fullSpeedFwd  = maxV[Ax.FWD.ordinal()]  * fullScale(Ax.FWD.ordinal());
        if (maxV[Ax.STR.ordinal()] > 1)  driveBase.fullSpeedStr  = maxV[Ax.STR.ordinal()]  * fullScale(Ax.STR.ordinal());
        if (maxV[Ax.TURN.ordinal()] > 1) driveBase.fullSpeedTurn = maxV[Ax.TURN.ordinal()] * fullScale(Ax.TURN.ordinal());
    }

    /** Clamp measured max speed / accel into [MIN, MAX]. The bounds are
     *  FULL-POWER equivalents, so each measurement is converted to full power,
     *  clamped, then converted back to the test power. Failed measurements
     *  (<= 1) are left alone so they still fall through to the FALLBACK_*
     *  values instead of being "rescued" up to the minimum. */
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

    /** Max speed to OUTPUT for an axis (full-power scaled when that mode is on). */
    private double outV(int idx, double fallback) {
        double v = positiveOr(maxV[idx], fallback);
        return (fullPowerOutput && maxV[idx] > 1) ? v * fullScale(idx) : v;
    }

    /** Gains to OUTPUT. In full-power mode the model is re-run with the
     *  full-power accel-per-power (everything else - kS, tolerances, locked
     *  ts - stays as tuned), then the model is restored. */
    private double[] outputGains() {
        if (tunedGains == null) return null;
        if (!fullPowerOutput) return tunedGains;

        double f = model.aFwd, s = model.aStr, t = model.aTurn;
        if (maxA[Ax.FWD.ordinal()]  > 1) model.aFwd  = f * maxPower * fullScale(Ax.FWD.ordinal());
        if (maxA[Ax.STR.ordinal()]  > 1) model.aStr  = s * maxPower * fullScale(Ax.STR.ordinal());
        if (maxA[Ax.TURN.ordinal()] > 1) model.aTurn = t * maxPower * fullScale(Ax.TURN.ordinal());
        double[] g = model.gains();
        model.aFwd = f; model.aStr = s; model.aTurn = t;
        return g;
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
        sIter = 0;
        searching = true;
        evals.clear();
        stage = Stage.TS;
        gsA = Math.log(sIdx == 2 ? TS_LO_H : TS_LO_T);
        gsB = Math.log(sIdx == 2 ? TS_HI_H : TS_HI_T);
        gsC = gsB - GOLD * (gsB - gsA);
        gsD = gsA + GOLD * (gsB - gsA);
        gsFc = Double.NaN;
        gsFd = Double.NaN;
        gsPendingC = true;
        runProbeAt(Math.exp(gsC), 1.0, END_DEFAULT);
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
                runProbeAt(Math.exp(gsD), 1.0, END_DEFAULT);
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
                runProbeAt(Math.exp(gsC), 1.0, END_DEFAULT);
            } else {
                gsA = gsC; gsC = gsD; gsFc = gsFd;
                gsD = gsA + GOLD * (gsB - gsA);
                gsPendingC = false;
                runProbeAt(Math.exp(gsD), 1.0, END_DEFAULT);
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
        if (stage == Stage.ZETA && sIdx < 2) startTweakStage(Stage.END);
        else {
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
        double[] tries = (s == Stage.ZETA) ? ZETA_TRY : END_TRY;
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
        double x1 = px(p1[k], z), x2 = px(p2[k], z), x3 = px(p3[k], z);
        double y1 = p1[3], y2 = p2[3], y3 = p3[3];
        double den = (x1 - x2) * (x1 - x3) * (x2 - x3);
        if (Math.abs(den) < 1e-9) return null;
        double a = (x3 * (y2 - y1) + x2 * (y1 - y3) + x1 * (y3 - y2)) / den;
        double b = (x3 * x3 * (y1 - y2) + x2 * x2 * (y3 - y1) + x1 * x1 * (y2 - y3)) / den;
        if (a <= 0) return null;
        double xv = -b / (2 * a);
        double lo = z ? ZETA_LO : Math.log(END_LO), hi = z ? ZETA_HI : Math.log(END_HI);
        xv = clamp(xv, lo, hi);
        double minGap = 0.08;
        if (Math.abs(xv - x1) < minGap || Math.abs(xv - x2) < minGap || Math.abs(xv - x3) < minGap) return null;
        double[] c = stageBase.clone();
        c[k] = z ? xv : Math.exp(xv);
        return c;
    }

    /** Search coordinate: zeta directly, endpoint ratio in log space. */
    private static double px(double v, boolean zeta) { return zeta ? v : Math.log(v); }

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
        return costInt + costLastF * Math.max(0, COST_HORIZON - costLastT) + W_TWITCH * reversals;
    }

    private double penalty(double err) {
        double r = err / (probeIsHeading() ? BALANCE_H : BALANCE_T);
        return 1 - Math.exp(-r * r);
    }

    private void runProbeAt(double ts, double zeta, double end) {
        curTs = ts;
        curZeta = zeta;
        curEnd = end;
        applyParams(ts, zeta, end);
        buildTunedFollower();
        beginPath(PROBE[sIdx]);
        state = State.SEARCH_PROBE;
    }

    private void lockAxis(double[] e) {
        lockedTs[sIdx] = e[0];
        applyParams(e[0], e[1], e[2]);
        model.loopDt = loopDt;
        // from here on, drive home with the tuned gains - much faster than
        // the conservative safe set
        safeFollow = new Follower(buildConfig(model.gains(),
                positiveOr(maxV[Ax.FWD.ordinal()], FALLBACK_XV),
                positiveOr(maxV[Ax.STR.ordinal()], FALLBACK_YV),
                profileAccel(false), profileAccel(false)));
        buildTunedFollower();
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

    private void applyParams(double ts, double zeta, double end) {
        if (sIdx == 0) { model.tsX = ts; model.zetaX = zeta; model.endX = end; }
        else if (sIdx == 1) { model.tsY = ts; model.zetaY = zeta; model.endY = end; }
        else { model.tsH = ts; model.zetaH = zeta; }
    }

    private double axisTol(int i) { return i == 0 ? model.tolX : i == 1 ? model.tolY : model.tolH; }

    private void buildTunedFollower() {
        model.loopDt = loopDt;
        tunedGains = model.gains();
        follow = new Follower(buildConfig(tunedGains,
                positiveOr(maxV[Ax.FWD.ordinal()], FALLBACK_XV),
                positiveOr(maxV[Ax.STR.ordinal()], FALLBACK_YV),
                profileAccel(false), profileAccel(false)));
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
        FollowerPrimer.primeHeading(follow, targetHeading, odometry.Heading());

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

    private double balance() { return probeIsHeading() ? BALANCE_H : BALANCE_T; }

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
        returnTries = 0;
        rebuildReturnPath();
    }

    private void rebuildReturnPath() {
        final Vector2D[] pts = new Vector2D[]{
                new Vector2D(odometry.X(), odometry.Y()),
                new Vector2D(CX, CY)
        };
        SectionBuilder[] section = new SectionBuilder[]{ () -> addPts(pts) };
        paths.addNewPath("backPath");
        paths.buildPath(section);
        safeFollow.setPath(paths.returnPath("backPath"));
        safeFollow.usePathHeadings(false);
        safeFollow.holdPositionAtPathEnd(true);
        FollowerPrimer.primeHeading(safeFollow, 0, odometry.Heading());
    }

    /** @return true only when ACTUALLY near home (retries the return path if a
     *  timeout leaves the robot far away - starting the next run from a bad
     *  position was what marched the robot off the field). */
    private boolean returnHome() {
        drive(safeFollow, 0);
        boolean stopped = Math.hypot(odometry.getXVelocity(), odometry.getYVelocity()) < stopEpsT;
        boolean home = safeFollow.isFinished(8, 8);
        double distHome = Math.hypot(odometry.X() - CX, odometry.Y() - CY);
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
        FollowerPrimer.prime(f, odometry.Heading(), odometry.X(), odometry.Y());
        RobotPower cp = f.followPathAuto(heading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
        driveBase.queueCommand(driveBase.drivePowers(
                clamp(cp.getVertical(), -maxPower, maxPower),
                clamp(cp.getHorizontal(), -maxPower, maxPower),
                clamp(cp.getPivot(), -maxPower, maxPower)));
    }

    private void adjustLimits() {
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

    private double degPerSec() { return odometry.getHeadingVelocityDeg(); }

    private static RobotConfig buildConfig(double[] p, double maxXV, double maxYV, double maxXA, double maxYA) {
        return new RobotConfig()
                .setXOnPathPD(p[0], p[1])
                .setYOnPathPD(p[2], p[3])
                .setXLastAdjustmentPD(p[4], p[5])
                .setYLastAdjustmentPD(p[6], p[7])
                .setFastHeadingPD(p[8], p[9])
                .setSlowHeadingPD(p[10], p[11])
                .setRobotConstants(maxXV, maxYV, maxXA, maxYA);
    }

    private static double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
    private static double wrap360(double a) { a %= 360; if (a < 0) a += 360; return a; }
    private static double wrap180(double a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }

    // ============================= TELEMETRY / LOGGING =============================
    private String modeLabel() {
        return fullPowerOutput
                ? "FULL POWER (scaled up from tests at " + String.format("%.2f", maxPower) + ")"
                : "AS TESTED (at " + String.format("%.2f", maxPower) + " power)";
    }

    private void drawTelemetry() {
        telemetry.addLine("dpad_up=START  B=STOP+SAVE  X=SKIP STEP");
        telemetry.addLine("Y = TOGGLE OUTPUT MODE   (idle: dpad L/R = test power, bumpers = area)");
        telemetry.addData("OUTPUT MODE (press Y)", modeLabel());
        telemetry.addData("state", state);
        telemetry.addData("TEST POWER (dpad L/R)", String.format("%.2f", maxPower));
        telemetry.addData("TUNING AREA (bumpers)", String.format("%.0f cm  (path scale %.2f)", areaSize, pathScale()));
        telemetry.addData("noise pos/hdg/velT/velH", String.format("%.2f / %.2f / %.1f / %.1f",
                posNoise, hdgNoise, velNoiseT, velNoiseH));
        telemetry.addData("breakaway F/S/T", String.format("%.2f / %.2f / %.2f",
                ks[0], ks[1], ks[2]));
        telemetry.addData("tol X/Y/H (derived)", String.format("%.1f / %.1f / %.1f",
                model.tolX, model.tolY, model.tolH));
        telemetry.addData("maxV F/S/T (at test power)", String.format("%.0f / %.0f / %.0f  plateau %b/%b/%b",
                maxV[0], maxV[1], maxV[2], plateauOk[0], plateauOk[1], plateauOk[2]));
        telemetry.addData("A F/S/T (regression)", String.format("%.0f / %.0f / %.0f",
                maxA[0], maxA[1], maxA[2]));
        if (vClamped[0] || vClamped[1] || vClamped[2] || aClamped[0] || aClamped[1] || aClamped[2])
            telemetry.addData("!! BOUNDS HIT (V F/S/T, A F/S/T)", String.format("%b/%b/%b, %b/%b/%b",
                    vClamped[0], vClamped[1], vClamped[2], aClamped[0], aClamped[1], aClamped[2]));
        telemetry.addData("brake decel F/S/T (0 power)", String.format("%.0f / %.0f / %.0f   profile accel %.0f",
                brakeDecel[0], brakeDecel[1], brakeDecel[2], profileAccel(false)));
        telemetry.addData("loop dt", String.format("%.1f ms   max wheel speed %.2f", loopDt * 1000,
                driveBase.maxWheelSpeedFraction()));
        if (state == State.SEARCH_PROBE || state == State.SEARCH_RETURN)
            telemetry.addData("search", String.format("%s %s  ts %.2f  zeta %.2f  end %.2f  run %d  last cost %.2f",
                    axisName(sIdx), stage, curTs, curZeta, curEnd, sIter + 1, lastCost));
        if (state == State.SEARCH_PROBE)
            telemetry.addData("this run", String.format("in area @%.2fs  stopped @%.2fs  reversals %d  fine exits %d  cost so far %.2f",
                    tCoarse, settledAt, reversals, fineExits, costInt));
        if (state == State.VERIFY_RUN || state == State.VERIFY_RETURN)
            telemetry.addData("verify path", (batteryIdx + 1) + "/" + BATTERY.length);
        telemetry.addData("locked ts X/Y/H", String.format("%.2f / %.2f / %.2f",
                lockedTs[0], lockedTs[1], lockedTs[2]));
        telemetry.addData("zeta X/Y/H  end X/Y", String.format("%.2f / %.2f / %.2f   %.2f / %.2f",
                model.zetaX, model.zetaY, model.zetaH, model.endX, model.endY));
        telemetry.addData("pos", String.format("%.0f,%.0f  h%.0f",
                odometry.X(), odometry.Y(), odometry.Heading()));
        if (tunedGains != null) {
            telemetry.addLine("gains (as tested): " + fmt(tunedGains));
            if (fullPowerOutput) telemetry.addLine("gains (OUTPUT, full power): " + fmt(outputGains()));
            telemetry.addData("OUTPUT consts maxXV/YV/XA/YA", String.format("%.0f / %.0f / %.0f / %.0f",
                    outV(Ax.FWD.ordinal(), FALLBACK_XV), outV(Ax.STR.ordinal(), FALLBACK_YV),
                    profileAccel(fullPowerOutput), profileAccel(fullPowerOutput)));
        }
        telemetry.addData("best cost X/Y/H", String.format("%.2f / %.2f / %.2f",
                bestCost[0], bestCost[1], bestCost[2]));
        if (state == State.FINISHED) telemetry.addLine(">>> DONE - saved to tune_best.txt (" + (fullPowerOutput ? "FULL POWER" : "AS TESTED") + ") - Y switches mode + re-saves <<<");
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

    private void saveBest() {
        try {
            File dir = new File("/sdcard/FIRST/datalogs");
            if (!dir.exists()) dir.mkdirs();
            FileWriter w = new FileWriter(new File(dir, "tune_best.txt"), false);
            double[] og = outputGains();
            double[] b = og != null ? og : SAFE_GAINS;
            w.write("// PathTuner (self tuning) result - everything below was MEASURED\n");
            w.write("// OUTPUT MODE: " + modeLabel() + "\n");
            w.write(String.format("// odometry noise  pos=%.2fcm hdg=%.2fdeg  vel=%.1fcm/s %.1fdeg/s\n",
                    posNoise, hdgNoise, velNoiseT, velNoiseH));
            w.write(String.format("// breakaway power  fwd=%.2f strafe=%.2f turn=%.2f\n",
                    model.ksFwd, model.ksStr, model.ksTurn));
            w.write(String.format("// derived tolerances  X=%.1f Y=%.1f cm  H=%.1f deg\n",
                    model.tolX, model.tolY, model.tolH));
            w.write(String.format("// measured maxV (at test power)  fwd=%.0f strafe=%.0f turn=%.0f  [plateau %b/%b/%b]\n",
                    maxV[0], maxV[1], maxV[2], plateauOk[0], plateauOk[1], plateauOk[2]));
            w.write(String.format("// accel per power (regression)  fwd=%.0f strafe=%.0f turn=%.0f\n",
                    model.aFwd, model.aStr, model.aTurn));
            w.write(String.format("// brake decel (0 power)  fwd=%.0f strafe=%.0f turn=%.0f  -> profile accel %.0f\n",
                    brakeDecel[0], brakeDecel[1], brakeDecel[2], profileAccel(fullPowerOutput)));
            w.write(String.format("// locked ts  X=%.2f Y=%.2f H=%.2f  zeta %.2f/%.2f/%.2f  end %.2f/%.2f  best cost %.2f / %.2f / %.2f\n",
                    lockedTs[0], lockedTs[1], lockedTs[2], model.zetaX, model.zetaY, model.zetaH,
                    model.endX, model.endY, bestCost[0], bestCost[1], bestCost[2]));
            w.write(String.format("// D gains are PER LOOP, tuned at loop dt %.1f ms - retune if your OpMode loop is much slower/faster\n",
                    model.loopDt * 1000));
            w.write(String.format("// DriveBase: fullSpeedFwd=%.0f fullSpeedStr=%.0f fullSpeedTurn=%.0f\n",
                    driveBase.fullSpeedFwd, driveBase.fullSpeedStr, driveBase.fullSpeedTurn));
            w.write("new RobotConfig()\n");
            w.write(String.format("    .setXOnPathPD(%.5f, %.5f)\n", b[0], b[1]));
            w.write(String.format("    .setYOnPathPD(%.5f, %.5f)\n", b[2], b[3]));
            w.write(String.format("    .setXLastAdjustmentPD(%.5f, %.5f)\n", b[4], b[5]));
            w.write(String.format("    .setYLastAdjustmentPD(%.5f, %.5f)\n", b[6], b[7]));
            w.write(String.format("    .setFastHeadingPD(%.5f, %.5f)\n", b[8], b[9]));
            w.write(String.format("    .setSlowHeadingPD(%.5f, %.5f)\n", b[10], b[11]));
            w.write(String.format("    .setRobotConstants(%.0f, %.0f, %.0f, %.0f);\n",
                    outV(Ax.FWD.ordinal(), FALLBACK_XV),
                    outV(Ax.STR.ordinal(), FALLBACK_YV),
                    profileAccel(fullPowerOutput),
                    profileAccel(fullPowerOutput)));
            w.close();
        } catch (IOException ignored) {}
    }
}