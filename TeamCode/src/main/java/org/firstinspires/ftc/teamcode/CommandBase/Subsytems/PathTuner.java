package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;


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
 *                  noise-spike gains). Braking is a non-blocking state.
 *   4. SEARCH    - per axis, shrink the settling target ts while probe runs
 *                  pass. On the first fail, back off with margin (last pass
 *                  x1.2) and CONFIRM with a fresh run before locking - gains
 *                  land safely inside the stable region, not on its edge.
 *   5. VERIFY    - battery of paths with the locked gains, logged to CSV.
 *
 * Gains come from pole placement at zeta=1 (see ModelTuner) - Kd is always
 * re-derived from the final Kp so clamping can never produce an underdamped
 * (wobbly) pair.
 *
 * Place the robot mid-field (180,180) facing 0, dpad_up to start.
 * Outputs: /sdcard/FIRST/datalogs/tune_best.txt, tune_paths.csv.
 * Controls: dpad_up = start, B = stop & save, X = skip current step.
 */
@TeleOp(name = "PathTuner (self tuning)", group = "tuning")
public class PathTuner extends OpModeEX {

    // ---- internal constants (measurement procedure, not tuning knobs) ----
    private static final double NOISE_TIME      = 1.0;   // s standstill sampling
    private static final double RAMP_RATE       = 0.25;  // power/s breakaway ramp
    private static final double RAMP_MAX_POWER  = 0.8;   // give up above this
    private static final double LURCH_TIME      = 0.35;  // s to watch post-breakaway drift
    private static final double MAXVA_MAX_TIME  = 1.2;   // s full-power cap
    private static final double MAXVA_MARGIN    = 40;    // cm inner box for full-power runs
    private static final int    PLATEAU_N       = 5;
    private static final double PLATEAU_TOL     = 0.04;
    private static final double BRAKE_POWER     = 0.5;
    private static final double BRAKE_MAX_TIME  = 0.5;   // s
    private static final double SETTLE_TIMEOUT  = 3.5;   // s per probe run
    private static final double RETURN_TIMEOUT  = 6.0;   // s per return attempt
    private static final int    RETURN_RETRIES  = 3;     // rebuild+retry if not home
    private static final double HOME_OK_DIST    = 30;    // cm - close enough to start a path
    private static final int    SETTLE_N        = 5;     // consecutive in-zone+stopped samples
    private static final int    OVERSHOOT_N     = 4;     // consecutive past-tol samples = real
    private static final double TS_START_T      = 0.90;  // conservative search starts
    private static final double TS_START_H      = 0.70;
    private static final double TS_FLOOR        = 0.12;
    private static final double TS_DOWN         = 0.75;  // shrink on pass
    private static final double LOCK_MARGIN     = 1.20;  // back off from edge before locking
    private static final int    MAX_ITER        = 16;
    private static final double SAMPLE_DT_MIN   = 0.02;
    private static final int    MAX_SAMPLES     = 256;

    // field / safety
    private static final double CX = 180, CY = 180;
    private static final double BOX_MIN = 55, BOX_MAX = 305;

    // known-good gains ONLY for driving home between steps (never tuned/saved).
    // ~2x the old conservative set - returns were crawling. Only used until
    // the first axis is tuned; after that returns run on the tuned gains.
    private static final double[] SAFE_GAINS = {
            0.16, 0.008,  0.3, 0.006,
            0.05, 0.008,  0.05, 0.014,
            0.02, 0.001,  0.02, 0.003
    };
    private static final double FALLBACK_XV = 130, FALLBACK_YV = 181;
    private static final double FALLBACK_XA = 650, FALLBACK_YA = 700;

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
    private int sIdx = 0;                    // 0=X(STR), 1=Y(FWD), 2=H(TURN)
    private double curTs, lastPassTs;
    private boolean anyPass, confirming;
    private int confirmTries = 0;
    private int sIter;
    private boolean searching = false;
    private final double[] lockedTs = { Double.NaN, Double.NaN, Double.NaN };
    private final double[] firstSettle = { Double.NaN, Double.NaN, Double.NaN };
    private final double[] bestSettle  = { Double.NaN, Double.NaN, Double.NaN };

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
        driveBase.tele = false;
        driveBase.speed = 1;

        genConfig = buildConfig(SAFE_GAINS, FALLBACK_XV, FALLBACK_YV, FALLBACK_XA, FALLBACK_YA);
        paths = new PathsManager(genConfig);
        safeFollow = new Follower(genConfig);

        telemetry.addLine("PathTuner (self tuning). Robot at CENTRE, facing 0.");
        telemetry.addLine("No values to set - everything is measured.");
        telemetry.addLine("dpad_up = start,  B = stop+save,  X = skip step");
    }

    @Override
    public void loopEX() {
        boolean start = currentGamepad1.dpad_up && !lastGamepad1.dpad_up;
        boolean stop  = currentGamepad1.b && !lastGamepad1.b;
        boolean skip  = currentGamepad1.x && !lastGamepad1.x;

        if (stop && state != State.IDLE && state != State.FINISHED) {
            if (tunedGains != null) saveBest();
            stopDrive();
            state = State.FINISHED;
        }

        odometry.queueCommand(odometry.update);

        switch (state) {
            case IDLE:
                stopDrive();
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
                if (returnHome()) searchDecision();
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
        rampPower = Math.min(RAMP_MAX_POWER, phaseTimer.seconds() * RAMP_RATE);
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

        if (moved || rampPower >= RAMP_MAX_POWER || skip) {
            ks[axIdx] = moved ? rampPower : RAMP_MAX_POWER * 0.5; // no-move fallback
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
        model.tolX = clamp(1.5 * quantum[Ax.STR.ordinal()], Math.max(2.5, posNoise * 4), 5.0);
        model.tolY = clamp(1.5 * quantum[Ax.FWD.ordinal()], Math.max(2.5, posNoise * 4), 5.0);
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
        boolean boxHit = outsideBox(MAXVA_MARGIN);
        boolean timeUp = t >= MAXVA_MAX_TIME;

        if (boxHit || timeUp || skip) { finishMaxVaRun(false); return; }

        applyAxisPower(Ax.values()[axIdx], 1.0);

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
        maxA[axIdx] = rampSlope(vBest);   // least-squares, not sample-to-sample
        phaseTimer.reset();
        state = State.MAXVA_BRAKE;
    }

    /** Least-squares slope of v(t) over the ramp portion (10%..70% of peak).
     *  Immune to single-sample odometry noise, unlike a peak dV/dt. */
    private double rampSlope(double vPeak) {
        double lo = 0.10 * vPeak, hi = 0.70 * vPeak;
        double n = 0, st = 0, sv = 0, stt = 0, stv = 0;
        for (double[] s : samples) {
            if (s[1] < lo || s[1] > hi) continue;
            n++; st += s[0]; sv += s[1]; stt += s[0] * s[0]; stv += s[0] * s[1];
        }
        if (n < 4) return 0;                       // not enough ramp data
        double denom = n * stt - st * st;
        if (Math.abs(denom) < 1e-9) return 0;
        return (n * stv - st * sv) / denom;        // cm/s^2 (or deg/s^2) at power 1.0
    }

    /** Non-blocking reverse-power brake until actually stopped. */
    private void runMaxVaBrake() {
        double v = (Ax.values()[axIdx] == Ax.TURN)
                ? Math.abs(degPerSec())
                : Math.hypot(odometry.getXVelocity(), odometry.getYVelocity());
        double eps = (Ax.values()[axIdx] == Ax.TURN) ? stopEpsH : stopEpsT;
        if (v < eps || phaseTimer.seconds() > BRAKE_MAX_TIME) {
            stopDrive();
            buildReturnPath();
            state = State.MAXVA_RETURN;
            phaseTimer.reset();
        } else {
            applyAxisPower(Ax.values()[axIdx], -BRAKE_POWER);
        }
    }

    private void finishMaxVa() {
        // accel-per-power straight from the regression (power was 1.0)
        model.aFwd  = positiveOr(maxA[Ax.FWD.ordinal()],  FALLBACK_YA);
        model.aStr  = positiveOr(maxA[Ax.STR.ordinal()],  FALLBACK_XA);
        model.aTurn = positiveOr(maxA[Ax.TURN.ordinal()], 300);

        double xv = positiveOr(maxV[Ax.STR.ordinal()], FALLBACK_XV);
        double yv = positiveOr(maxV[Ax.FWD.ordinal()], FALLBACK_YV);
        double xa = positiveOr(maxA[Ax.STR.ordinal()], FALLBACK_XA);
        double ya = positiveOr(maxA[Ax.FWD.ordinal()], FALLBACK_YA);

        // rebuild path generation + safe follower with real constants
        genConfig = buildConfig(SAFE_GAINS, xv, yv, xa, ya);
        paths = new PathsManager(genConfig);
        safeFollow = new Follower(genConfig);
    }

    private static double positiveOr(double v, double fallback) {
        return (Double.isNaN(v) || v <= 1) ? fallback : v;
    }

    // ============================= 4. SEARCH =============================
    private void beginSearchAxis(int i) {
        sIdx = i;
        curTs = (sIdx == 2) ? TS_START_H : TS_START_T;
        lastPassTs = Double.NaN;
        anyPass = false;
        confirming = false;
        sIter = 0;
        searching = true;
        applyTs(curTs);
        buildTunedFollower();
        beginPath(PROBE[sIdx]);
        state = State.SEARCH_PROBE;
    }

    private int invalidRuns = 0;
    private boolean invalidRun = false;

    private void searchDecision() {
        // started outside the box - position problem, not a gains verdict;
        // rerun the SAME ts without spending an iteration (bounded)
        if (invalidRun && invalidRuns < 3) {
            invalidRuns++;
            runProbeAt(curTs);
            return;
        }
        invalidRuns = 0;

        double err = (sIdx == 0) ? tr.finalErrX : (sIdx == 1) ? tr.finalErrY : tr.finalHdgErr;
        double tol = axisTol(sIdx);
        boolean pass = !tr.timedOut && !tr.leftBox && err <= tol && !sustainedOvershoot;
        // DIRECTION of the failure matters:
        //   too WEAK  (timed out / never reached tolerance, no overshoot)
        //       -> ts must go DOWN (stronger gains)
        //   too STRONG (overshoot / left the box)
        //       -> ts must go UP (weaker gains)
        // The old logic treated every fail as "too strong", so a weak start
        // spiralled weaker each iteration and the robot just crept in place.
        boolean tooStrong = tr.leftBox || sustainedOvershoot;

        if (sIter == 0) firstSettle[sIdx] = tr.settleTime;
        sIter++;

        if (confirming) {
            if (pass) { lockAxis(curTs); return; }
            confirmTries++;
            // Don't ping-pong between "too weak" and "too strong" forever -
            // after two failed confirms just lock the last pass with margin.
            if (confirmTries >= 2 || sIter >= MAX_ITER) {
                lockAxis(anyPass ? lastPassTs * LOCK_MARGIN : clampTs(curTs));
                return;
            }
            curTs = tooStrong ? curTs * LOCK_MARGIN : curTs * TS_DOWN;
            runProbeAt(clampTs(curTs));
            return;
        }

        if (pass) {
            lastPassTs = curTs;
            anyPass = true;
            bestSettle[sIdx] = tr.settleTime;
            curTs *= TS_DOWN;
            if (curTs < TS_FLOOR || sIter >= MAX_ITER) {
                // hit the floor while still passing - confirm at margin and lock
                confirming = true;
                confirmTries = 0;
                curTs = Math.max(TS_FLOOR, lastPassTs) * LOCK_MARGIN;
                runProbeAt(curTs);
                return;
            }
            runProbeAt(curTs);
        } else if (anyPass) {
            // first fail after passes: back off with margin, then CONFIRM
            confirming = true;
            confirmTries = 0;
            curTs = lastPassTs * LOCK_MARGIN;
            runProbeAt(curTs);
        } else {
            // never passed yet - move in the direction the failure demands
            curTs = tooStrong ? curTs / TS_DOWN : curTs * TS_DOWN;
            if (sIter >= MAX_ITER || curTs > 2.5 || curTs < TS_FLOOR) {
                lockAxis(clampTs(curTs));
                return;
            }
            runProbeAt(curTs);
        }
    }

    private static double clampTs(double ts) {
        return ts < TS_FLOOR ? TS_FLOOR : (ts > 2.5 ? 2.5 : ts);
    }

    private void runProbeAt(double ts) {
        applyTs(ts);
        buildTunedFollower();
        beginPath(PROBE[sIdx]);
        state = State.SEARCH_PROBE;
    }

    private void lockAxis(double ts) {
        lockedTs[sIdx] = ts;
        applyTs(ts);
        // from here on, drive home with the tuned gains - much faster than
        // the conservative safe set
        safeFollow = new Follower(buildConfig(model.gains(),
                positiveOr(maxV[Ax.STR.ordinal()], FALLBACK_XV),
                positiveOr(maxV[Ax.FWD.ordinal()], FALLBACK_YV),
                positiveOr(maxA[Ax.STR.ordinal()], FALLBACK_XA),
                positiveOr(maxA[Ax.FWD.ordinal()], FALLBACK_YA)));
        if (sIdx >= 2) {
            buildTunedFollower();
            saveBest();
            searching = false;
            batteryIdx = 0;
            beginPath(BATTERY[0]);
            state = State.VERIFY_RUN;
        } else {
            beginSearchAxis(sIdx + 1);
        }
    }

    private void applyTs(double ts) {
        if (sIdx == 0) model.tsX = ts;
        else if (sIdx == 1) model.tsY = ts;
        else model.tsH = ts;
    }

    private double axisTol(int i) { return i == 0 ? model.tolX : i == 1 ? model.tolY : model.tolH; }

    private void buildTunedFollower() {
        tunedGains = model.gains();
        follow = new Follower(buildConfig(tunedGains,
                positiveOr(maxV[Ax.STR.ordinal()], FALLBACK_XV),
                positiveOr(maxV[Ax.FWD.ordinal()], FALLBACK_YV),
                positiveOr(maxA[Ax.STR.ordinal()], FALLBACK_XA),
                positiveOr(maxA[Ax.FWD.ordinal()], FALLBACK_YA)));
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
            rel[i][0] = it.rel[i][0] * sx;
            rel[i][1] = it.rel[i][1] * sy;
        }

        double[] last = rel[rel.length - 1];
        targetX = cx + last[0];
        targetY = cy + last[1];
        double len = Math.hypot(last[0], last[1]);
        dirX = len > 0 ? last[0] / len : 0;
        dirY = len > 0 ? last[1] / len : 0;
        targetHeading = wrap360(startHeading + it.hdgDelta);
        hdgDeltaSign = Math.signum(it.hdgDelta);

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
        phaseTimer.reset();
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

        // settled = inside the tolerance box AND slow, held for SETTLE_N
        // consecutive samples. During a probe only the PROBED axis is judged
        // at full tolerance - the other axes just need to be roughly sane
        // (3x tol). Demanding all three at full tolerance failed good runs on
        // cross-axis noise and drove the search toward mush gains.
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
        // "slow", not perfectly stopped - holdPositionAtPathEnd keeps making
        // micro-corrections, so a hard stop threshold never latches
        double vt = Math.hypot(odometry.getXVelocity(), odometry.getYVelocity());
        boolean stopped = vt < stopEpsT * 3 && Math.abs(degPerSec()) < stopEpsH * 3;

        if (inZone && stopped) {
            if (settleStreak == 0) settleEnterT = phaseTimer.seconds();
            settleStreak++;
            if (settleStreak >= SETTLE_N) {
                tr.settleTime = settleEnterT;
                tr.finalErrX = aX;
                tr.finalErrY = aY;
                tr.finalPosErr = Math.hypot(aX, aY);
                tr.finalHdgErr = aH;
                finishPath();
                return;
            }
        } else {
            settleStreak = 0;
        }

        if (phaseTimer.seconds() > SETTLE_TIMEOUT) {
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
        switch (axis) {
            case FWD:  driveBase.queueCommand(driveBase.drivePowers(p, 0, 0)); break;
            case STR:  driveBase.queueCommand(driveBase.drivePowers(0, 0, p)); break;
            case TURN: driveBase.queueCommand(driveBase.drivePowers(0, p, 0)); break;
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
        RobotPower cp = f.followPathAuto(heading, odometry.Heading(),
                odometry.X(), odometry.Y(),
                odometry.getXVelocity(), odometry.getYVelocity());
        driveBase.queueCommand(driveBase.drivePowers(cp));
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
        return x < BOX_MIN + margin || x > BOX_MAX - margin
                || y < BOX_MIN + margin || y > BOX_MAX - margin;
    }

    private double degPerSec() { return odometry.getHVelocity() * 180.0 / Math.PI; }

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
    private void drawTelemetry() {
        telemetry.addData("state", state);
        telemetry.addData("noise pos/hdg/velT/velH", String.format("%.2f / %.2f / %.1f / %.1f",
                posNoise, hdgNoise, velNoiseT, velNoiseH));
        telemetry.addData("breakaway F/S/T", String.format("%.2f / %.2f / %.2f",
                ks[0], ks[1], ks[2]));
        telemetry.addData("tol X/Y/H (derived)", String.format("%.1f / %.1f / %.1f",
                model.tolX, model.tolY, model.tolH));
        telemetry.addData("maxV F/S/T", String.format("%.0f / %.0f / %.0f  plateau %b/%b/%b",
                maxV[0], maxV[1], maxV[2], plateauOk[0], plateauOk[1], plateauOk[2]));
        telemetry.addData("A F/S/T (regression)", String.format("%.0f / %.0f / %.0f",
                maxA[0], maxA[1], maxA[2]));
        if (state == State.SEARCH_PROBE || state == State.SEARCH_RETURN)
            telemetry.addData("search", axisName(sIdx) + "  ts=" + String.format("%.2f", curTs)
                    + "  iter=" + sIter + (confirming ? "  CONFIRMING" : ""));
        if (state == State.VERIFY_RUN || state == State.VERIFY_RETURN)
            telemetry.addData("verify path", (batteryIdx + 1) + "/" + BATTERY.length);
        telemetry.addData("locked ts X/Y/H", String.format("%.2f / %.2f / %.2f",
                lockedTs[0], lockedTs[1], lockedTs[2]));
        telemetry.addData("pos", String.format("%.0f,%.0f  h%.0f",
                odometry.X(), odometry.Y(), odometry.Heading()));
        if (tunedGains != null) telemetry.addLine("gains: " + fmt(tunedGains));
        for (int i = 0; i < 3; i++) {
            if (!Double.isNaN(firstSettle[i]) && !Double.isNaN(bestSettle[i]))
                telemetry.addData("settle " + axisName(i), String.format("%.2fs -> %.2fs",
                        firstSettle[i], bestSettle[i]));
        }
        if (state == State.FINISHED) telemetry.addLine(">>> DONE - saved to tune_best.txt <<<");
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
            double[] b = tunedGains != null ? tunedGains : SAFE_GAINS;
            w.write("// PathTuner (self tuning) result - everything below was MEASURED\n");
            w.write(String.format("// odometry noise  pos=%.2fcm hdg=%.2fdeg  vel=%.1fcm/s %.1fdeg/s\n",
                    posNoise, hdgNoise, velNoiseT, velNoiseH));
            w.write(String.format("// breakaway power  fwd=%.2f strafe=%.2f turn=%.2f\n",
                    model.ksFwd, model.ksStr, model.ksTurn));
            w.write(String.format("// derived tolerances  X=%.1f Y=%.1f cm  H=%.1f deg\n",
                    model.tolX, model.tolY, model.tolH));
            w.write(String.format("// measured maxV  fwd=%.0f strafe=%.0f turn=%.0f  [plateau %b/%b/%b]\n",
                    maxV[0], maxV[1], maxV[2], plateauOk[0], plateauOk[1], plateauOk[2]));
            w.write(String.format("// accel per power (regression)  fwd=%.0f strafe=%.0f turn=%.0f\n",
                    model.aFwd, model.aStr, model.aTurn));
            w.write(String.format("// locked ts  X=%.2f Y=%.2f H=%.2f  (zeta=%.2f, lock margin %.2fx)\n",
                    lockedTs[0], lockedTs[1], lockedTs[2], model.zeta, LOCK_MARGIN));
            w.write("new RobotConfig()\n");
            w.write(String.format("    .setXOnPathPD(%.5f, %.5f)\n", b[0], b[1]));
            w.write(String.format("    .setYOnPathPD(%.5f, %.5f)\n", b[2], b[3]));
            w.write(String.format("    .setXLastAdjustmentPD(%.5f, %.5f)\n", b[4], b[5]));
            w.write(String.format("    .setYLastAdjustmentPD(%.5f, %.5f)\n", b[6], b[7]));
            w.write(String.format("    .setFastHeadingPD(%.5f, %.5f)\n", b[8], b[9]));
            w.write(String.format("    .setSlowHeadingPD(%.5f, %.5f)\n", b[10], b[11]));
            w.write(String.format("    .setRobotConstants(%.0f, %.0f, %.0f, %.0f);\n",
                    positiveOr(maxV[Ax.STR.ordinal()], FALLBACK_XV),
                    positiveOr(maxV[Ax.FWD.ordinal()], FALLBACK_YV),
                    positiveOr(maxA[Ax.STR.ordinal()], FALLBACK_XA),
                    positiveOr(maxA[Ax.FWD.ordinal()], FALLBACK_YA)));
            w.close();
        } catch (IOException ignored) {}
    }
}
