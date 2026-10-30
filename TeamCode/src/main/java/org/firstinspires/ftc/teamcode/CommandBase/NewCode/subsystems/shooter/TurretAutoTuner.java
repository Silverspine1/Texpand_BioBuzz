package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Measures the turret and picks controller gains. Call step() once per loop.
 *
 * 1. Deadband: ramp power slowly until the turret starts to move.
 * 2. Sweep: open-loop runs at several powers, both directions -> kS, kV, response time.
 * 3. Gain limit: small closed-loop steps with kP rising until the turret oscillates (Ku).
 * 4. Search: candidates well below Ku, scored on settle time, overshoot and tracking.
 * 5. Verify: re-test the winner; back gains off until it is stable, or abort without a result.
 *
 * Every closed-loop test stops the turret immediately if it oscillates or leaves the safe range.
 */
public class TurretAutoTuner {

    public interface Io {
        /** Turret angle, deg, CCW positive. */
        double angle();

        /** Turret velocity, deg/s. */
        double velocity();

        /** Open-loop power added to the 0.5 servo neutral. */
        void setPower(double power);

        /** Closed-loop move using the given gains. */
        void moveTo(double angle, TurretTune gains);
    }

    public enum Stage { DEADBAND, SWEEP, MODEL, GAIN_LIMIT, SEARCH, VERIFY, DONE, ABORTED }

    private enum State {
        KS_WAIT_STILL, KS_RAMP,
        GOTO, SWEEP_REST, SWEEP_RUN,
        MODEL_SEARCH, TRIAL_MOVE, TRIAL_TRACK, BRAKE,
        FINISH_MOVE, DONE, ABORTED
    }

    private enum Phase { GAIN_LIMIT, SEARCH, VERIFY }

    public double safeRange = 110;
    public double abortMargin = 25;
    public double maxPower = 0.5;
    public double searchAngle = 70;

    private static final double KS_RAMP_RATE = 0.03;
    private static final double KS_RAMP_LIMIT = 0.3;
    private static final int KS_RAMPS = 4;
    // Speed saturates early on these servos (about 0.1 of command), so most levels sit at the low end.
    private static final double[] SWEEP_FRACTIONS = {0.02, 0.05, 0.10, 0.18, 0.30, 0.50, 1.0};
    private static final double REVERSAL_MIN_ERROR = 2.5;
    private static final double SATURATION_STEP = 0.08;
    private static final double SATURATED_FRACTION = 0.92;
    private static final double ASYMMETRY_LIMIT = 0.35;
    private static final double RUN_TIMEOUT = 6.0;
    private static final double GOTO_TOLERANCE = 5.0;
    private static final double SETTLE_TOLERANCE = TurretModelSearch.SETTLE_TOLERANCE;
    private static final double SETTLE_VELOCITY = 15;
    private static final double SETTLE_HOLD = 0.15;
    private static final double TRACK_SECONDS = 2.0;
    private static final double TRACK_AMPLITUDE = 25.0;
    private static final double TRACK_WEIGHT = 0.1;
    private static final double GAIN_STEP = 25.0;
    private static final double GAIN_START = 0.1;
    private static final double GAIN_GROWTH = 1.5;
    private static final double GAIN_MAX = 10.0;
    private static final int OSCILLATION_REVERSALS = 3;
    private static final double OSCILLATION_ERROR = 1.5;
    private static final int VERIFY_ATTEMPTS = 4;
    // round 1 = the model's finalists, rounds 2 and 3 = progressively gentler fallbacks if none of them is stable on the turret
    private static final int SEARCH_ROUNDS = 3;
    private static final long MODEL_SLICE_NANOS = 6_000_000L;
    private static final double SOFT_SETTLE_ERROR = 2.5;
    private static final double VERIFY_BACKOFF = 0.7;
    private static final double[][] SEARCH_KP_OF_KU = {{0.2, 0.3, 0.45}, {0.8, 1.2}};

    private final Io io;

    // ---- measured about this turret and its loop, for the plant model
    private double lastStepTime = -1;
    private double loopDtAvg = 0.017;
    private int loopDtCount = 0;
    private double prevAngle = Double.NaN;
    private double noiseSumSq = 0;
    private int noiseN = 0;
    private final List<double[]> decelSamples = new ArrayList<>();

    private static class RunTrace {
        final double speed;
        final double[][] samples;   // {seconds since command, degrees travelled}
        RunTrace(double speed, double[][] samples) {
            this.speed = speed;
            this.samples = samples;
        }
    }

    private final List<RunTrace> runTraces = new ArrayList<>();
    private TurretModelSearch modelSearch;
    private TurretPlantModel.Params plantParams;
    private boolean verified = false;
    private double modelBestScore = Double.NaN;

    private State state = State.KS_WAIT_STILL;
    private State afterGoto;
    private double stateStart = Double.NaN;
    private double now;
    private double angle;
    private double velocity;
    private String message = "starting";
    private String warning = "";

    private int ksDir = 1;
    private double ksStartAngle;
    private double ksFirstMoveTime = -1;
    private double ksFirstMovePower;
    private final List<Double> ksSamples = new ArrayList<>();
    private double kSRamp;

    private double gotoTarget;
    private boolean gotoMoving;
    private int gotoAttempts;
    private double stillSince = -1;

    private static class RunPlan {
        final double power;
        final int dir;
        RunPlan(double power, int dir) {
            this.power = power;
            this.dir = dir;
        }
    }

    private final List<RunPlan> sweepPlan = new ArrayList<>();
    private int runIndex = 0;
    private double runStartTime;
    private double runStartAngle;
    private final List<double[]> runSamples = new ArrayList<>();
    private boolean haveCut = false;
    private double cutAngle;
    private double cutVelocity;
    private double coastFactor = 0.12;
    private double gotoBoost = 0;
    private double moveStalledSince = -1;
    private double gotoStalledSince = -1;

    private final List<double[]> sweepPoints = new ArrayList<>();
    private final List<Double> t63Samples = new ArrayList<>();
    private final List<Double> moveDelaySamples = new ArrayList<>();

    private double fitKS, fitKV, fitR2, kVPositive, kVNegative;
    private double satPower = 0.5;
    private double responseTime, moveDelay, tau, topSpeed;
    private double reversalSpeed = 25;

    private TurretTune baseTune;
    private double modelKP;
    private double gainLimitKP = Double.NaN;
    private boolean gainLimitFound = false;

    // ---- trial runner
    private Phase phase;
    private TurretTune trialGains;
    private double trialStart;
    private double[] trialTargets;
    private boolean trialTrack;
    private int trialMoveIndex;
    private boolean moveStarted;
    private double moveTarget, moveStartTime, moveStartAngle, moveOvershoot, moveTimeout;
    private double settleSince = -1;
    private int reversals;
    private double lastVelocitySign;
    private int errorCrossings;
    private double lastErrorSign;
    private double errorPeakSinceCrossing;
    private double trialScoreSum, trialSettleSum, trialOvershootSum;
    private double trackStart, trackSumSq;
    private int trackCount;
    private String trialFailure;

    // ---- search
    private final List<TurretTune> candidates = new ArrayList<>();
    private int searchRound = 0;
    private int candidateIndex = 0;
    private TurretTune best;
    private double bestScore = Double.POSITIVE_INFINITY;
    private double bestSettle, bestOvershoot, bestTracking;
    private int candidatesTried = 0;
    private int candidatesFailed = 0;
    private int verifyAttempt = 0;
    private TurretTune result;

    public TurretAutoTuner(Io io) {
        this.io = io;
    }

    public void step(double timeSeconds) {
        now = timeSeconds;
        if (Double.isNaN(stateStart)) stateStart = now;

        angle = io.angle();
        velocity = io.velocity();

        double stepDt = lastStepTime < 0 ? 0 : now - lastStepTime;
        lastStepTime = now;
        if (stepDt > 0 && stepDt < 0.2) {
            loopDtAvg = loopDtCount == 0 ? stepDt : loopDtAvg + 0.02 * (stepDt - loopDtAvg);
            loopDtCount++;
        }
        if ((state == State.KS_WAIT_STILL || state == State.SWEEP_REST) && Math.abs(velocity) < 3
                && !Double.isNaN(prevAngle) && Double.isFinite(angle)) {
            noiseSumSq += (angle - prevAngle) * (angle - prevAngle);
            noiseN++;
        }
        prevAngle = angle;

        if (state == State.DONE) {
            return;
        }
        if (state == State.ABORTED) {
            io.setPower(0);
            return;
        }

        if (!Double.isFinite(angle) || !Double.isFinite(velocity)) {
            abort("encoder gave " + angle + " / " + velocity);
            return;
        }
        if (Math.abs(angle) > safeRange + abortMargin) {
            abort(String.format("turret at %.1f deg, outside the safe range", angle));
            return;
        }

        switch (state) {
            case KS_WAIT_STILL: ksWaitStill(); break;
            case KS_RAMP: ksRamp(); break;
            case GOTO: gotoStep(); break;
            case SWEEP_REST: sweepRest(); break;
            case SWEEP_RUN: sweepRun(); break;
            case MODEL_SEARCH: modelStep(); break;
            case TRIAL_MOVE: trialMove(); break;
            case TRIAL_TRACK: trialTrack(); break;
            case BRAKE: brake(); break;
            case FINISH_MOVE: finishMove(); break;
            default: break;
        }
    }

    // ---------------------------------------------------------------- deadband

    private void ksWaitStill() {
        io.setPower(0);
        message = String.format("deadband test %d/%d: waiting for turret to stop", ksSamples.size() + 1, KS_RAMPS);

        if (!isStill(0.3) && elapsed() < 1.5) return;

        if (ksSamples.isEmpty()) {
            ksDir = angle > 0 ? -1 : 1;
        }
        ksStartAngle = angle;
        ksFirstMoveTime = -1;
        enter(State.KS_RAMP);
    }

    private void ksRamp() {
        double power = KS_RAMP_RATE * elapsed();
        message = String.format("deadband test %d/%d: power %.3f", ksSamples.size() + 1, KS_RAMPS, power);

        if (power > KS_RAMP_LIMIT) {
            abort(String.format("turret didn't move up to power %.2f - check servo power and wiring", KS_RAMP_LIMIT));
            return;
        }
        io.setPower(ksDir * power);

        double moved = angle - ksStartAngle;
        if (Math.abs(moved) > 0.6) {
            if (ksFirstMoveTime < 0) {
                ksFirstMoveTime = now;
                ksFirstMovePower = power;
            }
        } else {
            ksFirstMoveTime = -1;
        }

        if (Math.abs(moved) > 2.0) {
            if (Math.signum(moved) != ksDir) {
                abort("+ power made the angle go DOWN - servo direction or encoder sign is reversed");
                return;
            }
            io.setPower(0);
            ksSamples.add(ksFirstMovePower);
            ksDir = -ksDir;

            if (ksSamples.size() < KS_RAMPS) {
                enter(State.KS_WAIT_STILL);
            } else {
                kSRamp = mean(ksSamples);
                buildSweepPlan();
                startGoto(-safeRange, State.SWEEP_REST);
            }
        }
    }

    // ---------------------------------------------------------------- open-loop positioning

    private void startGoto(double target, State next) {
        io.setPower(0);
        gotoTarget = target;
        afterGoto = next;
        haveCut = false;
        gotoMoving = true;
        gotoAttempts = 0;
        gotoBoost = 0;
        gotoStalledSince = -1;
        enter(State.GOTO);
    }

    /**
     * Open-loop move: drive, cut power early enough to coast in, retry if it misses. Each retry is gentler for the
     * overshoot, but never below breakaway power (that was a turret sitting still with the command at neutral), and a
     * turret that is not moving gets more power. Close counts as there: the trials start from wherever it is.
     */
    private void gotoStep() {
        double error = gotoTarget - angle;
        message = String.format("moving to %.0f deg (at %.1f)", gotoTarget, angle);

        if (elapsed() > 8 && Math.abs(error) < 25) {
            io.setPower(0);
            if (isStill(0.2)) enter(afterGoto);
            return;
        }
        if (elapsed() > 40) {
            abort("couldn't reach " + gotoTarget + " deg");
            return;
        }

        if (gotoMoving) {
            if (Math.abs(error) < 3 + coastFactor * Math.abs(velocity)) {
                io.setPower(0);
                gotoMoving = false;
                stillSince = -1;
                gotoStalledSince = -1;
                return;
            }
            if (Math.abs(velocity) < 5) {
                if (gotoStalledSince < 0) gotoStalledSince = now;
                if (now - gotoStalledSince > 0.35) {
                    gotoBoost = Math.min(maxPower - 1.1 * kSRamp, gotoBoost + 0.2 * kSRamp);
                    gotoStalledSince = now;
                }
            } else {
                gotoStalledSince = -1;
            }
            double scale = Math.pow(0.5, Math.min(gotoAttempts, 2));
            double power = 1.12 * kSRamp + gotoBoost + scale * (0.02 + Math.min(0.1, 0.003 * Math.abs(error)));
            io.setPower(Math.signum(error) * Math.min(maxPower, power));
            return;
        }

        io.setPower(0);
        if (!isStill(0.2)) return;

        if (Math.abs(error) < GOTO_TOLERANCE) {
            enter(afterGoto);
            return;
        }
        gotoAttempts++;
        gotoMoving = true;
        gotoStalledSince = -1;
    }

    // ---------------------------------------------------------------- sweep

    private void buildSweepPlan() {
        sweepPlan.clear();
        for (double f : SWEEP_FRACTIONS) {
            double p = kSRamp + f * (maxPower - kSRamp);
            sweepPlan.add(new RunPlan(p, 1));
            sweepPlan.add(new RunPlan(p, -1));
        }
        runIndex = 0;
    }

    private void sweepRest() {
        io.setPower(0);
        message = String.format("sweep run %d/%d: waiting for turret to stop", runIndex + 1, sweepPlan.size());

        if (!isStill(0.25) && elapsed() < 2.0) return;

        if (haveCut && Math.abs(cutVelocity) > 20) {
            double coast = Math.abs(angle - cutAngle);
            double measured = 1.5 * coast / Math.abs(cutVelocity);
            coastFactor = Math.max(0.03, Math.max(0.5 * coastFactor, measured));
            if (Math.abs(cutVelocity) > 100) decelSamples.add(new double[]{Math.abs(cutVelocity), coast});
        }
        haveCut = false;

        if (runIndex >= sweepPlan.size()) {
            analyzeSweep();
            return;
        }

        if (runIndex % 2 == 0 && runIndex >= 6 && saturationReached()) {
            runIndex = sweepPlan.size();
            analyzeSweep();
            return;
        }

        RunPlan plan = sweepPlan.get(runIndex);
        if (angle * plan.dir > -0.5 * safeRange) {
            startGoto(-plan.dir * safeRange, State.SWEEP_REST);
            return;
        }

        runStartTime = now;
        runStartAngle = angle;
        runSamples.clear();
        runSamples.add(new double[]{now, angle});
        io.setPower(plan.dir * plan.power);
        enter(State.SWEEP_RUN);
    }

    private void sweepRun() {
        RunPlan plan = sweepPlan.get(runIndex);
        message = String.format("sweep run %d/%d: power %.2f %s, %.0f deg/s",
                runIndex + 1, sweepPlan.size(), plan.power, plan.dir > 0 ? "+" : "-", velocity);

        io.setPower(plan.dir * plan.power);
        runSamples.add(new double[]{now, angle});

        double margin = 3 + coastFactor * Math.abs(velocity);
        boolean reachedEnd = angle * plan.dir >= safeRange - margin;
        boolean timedOut = elapsed() > RUN_TIMEOUT;
        boolean stalled = elapsed() > 1.5 && Math.abs(angle - runStartAngle) < 3;

        if (!reachedEnd && !timedOut && !stalled) return;

        io.setPower(0);
        haveCut = true;
        cutAngle = angle;
        cutVelocity = velocity;

        if (stalled) {
            warning = String.format("turret didn't move at power %.2f", plan.power);
        } else if (!analyzeRun(plan)) {
            return;
        }

        runIndex++;
        enter(State.SWEEP_REST);
    }

    /** Returns false if it aborted. */
    private boolean analyzeRun(RunPlan plan) {
        int n = runSamples.size();
        if (n < 6) {
            warning = "run too short to measure - loop time too slow?";
            return true;
        }

        double distance = (runSamples.get(n - 1)[1] - runStartAngle) * plan.dir;
        double[] fit = null;
        for (double fromFraction : new double[]{0.5, 0.3}) {
            List<double[]> tail = new ArrayList<>();
            for (double[] s : runSamples) {
                if ((s[1] - runStartAngle) * plan.dir >= fromFraction * distance) tail.add(s);
            }
            if (tail.size() >= 4 && tail.get(tail.size() - 1)[0] - tail.get(0)[0] >= 0.06) {
                // Median of pairwise slopes: one wild encoder sample can't move it (a least-squares line did).
                fit = new double[]{0, robustSlope(tail), 1};
                break;
            }
        }
        if (fit == null) {
            warning = String.format("not enough samples at power %.2f", plan.power);
            return true;
        }

        double speed = fit[1] * plan.dir;
        if (speed <= 0) {
            abort("+ power made the angle go DOWN - servo direction or encoder sign is reversed");
            return false;
        }
        sweepPoints.add(new double[]{plan.power, speed, plan.dir});

        // Keep the start of the run (distance travelled vs time since the command) to fit delay and acceleration later.
        if (speed > 60) {
            double[][] trace = new double[n][2];
            for (int i = 0; i < n; i++) {
                trace[i][0] = runSamples.get(i)[0] - runStartTime;
                trace[i][1] = (runSamples.get(i)[1] - runStartAngle) * plan.dir;
            }
            runTraces.add(new RunTrace(speed, trace));
        }

        if (plan.power >= kSRamp + 0.35 * (maxPower - kSRamp)) {
            double moveTime = -1;
            double reachTime = -1;
            for (int i = 1; i < n - 1; i++) {
                double[] a = runSamples.get(i - 1);
                double[] b = runSamples.get(i);
                double[] c = runSamples.get(i + 1);
                if (moveTime < 0 && Math.abs(b[1] - runStartAngle) > 1.0) {
                    moveTime = b[0] - runStartTime;
                }
                double v = (c[1] - a[1]) / (c[0] - a[0]) * plan.dir;
                if (v >= 0.63 * speed) {
                    reachTime = b[0] - runStartTime;
                    break;
                }
            }
            if (moveTime >= 0 && reachTime >= 0) {
                moveDelaySamples.add(moveTime);
                t63Samples.add(reachTime);
            }
        }
        return true;
    }

    private void analyzeSweep() {
        if (sweepPoints.size() < 4) {
            abort("only " + sweepPoints.size() + " usable sweep runs - increase safeRange or check the turret");
            return;
        }

        // Fit only below saturation: above it speed stops rising with power and a straight line is meaningless.
        double maxSpeed = 0;
        for (double[] p : sweepPoints) maxSpeed = Math.max(maxSpeed, p[1]);
        List<double[]> linear = new ArrayList<>();
        for (double[] p : sweepPoints) {
            if (p[1] < SATURATED_FRACTION * maxSpeed) linear.add(p);
        }
        if (linear.size() < 3) linear = sweepPoints;
        satPower = maxPower;
        for (double[] p : sweepPoints) {
            if (p[1] >= SATURATED_FRACTION * maxSpeed) satPower = Math.min(satPower, p[0]);
        }

        double[] fit = fitPowerVsSpeed(linear, 0);
        fitKS = fit[0];
        fitKV = fit[1];
        fitR2 = fit[2];
        kVPositive = directionKV(linear, 1, fit[0]);
        kVNegative = directionKV(linear, -1, fit[0]);

        if (!(fitKV > 0)) {
            abort("speed didn't increase with power - check the turret isn't jammed");
            return;
        }
        if (fitR2 < 0.7) {
            warning = String.format("speed vs power isn't very linear (R2 %.2f)", fitR2);
        }
        // Compare the plateau speeds, not the slopes: slopes come from a few runs near breakaway, where the same
        // servo legitimately starts differently each way (stiction), and read as a false "weak side".
        double topPositive = 0, topNegative = 0;
        for (double[] p : sweepPoints) {
            if (p[1] >= SATURATED_FRACTION * maxSpeed) {
                if (p[2] > 0) topPositive = Math.max(topPositive, p[1]);
                else topNegative = Math.max(topNegative, p[1]);
            }
        }
        if (topPositive > 0 && topNegative > 0
                && Math.abs(topPositive - topNegative) > ASYMMETRY_LIMIT * Math.max(topPositive, topNegative)) {
            warning = String.format("top speed differs by direction (%.0f vs %.0f deg/s)", topPositive, topNegative);
        }

        double kS = clamp(fitKS, 0.85 * kSRamp, kSRamp);

        responseTime = t63Samples.isEmpty() ? 0.08 : median(t63Samples);
        moveDelay = moveDelaySamples.isEmpty() ? 0.03 : median(moveDelaySamples);
        tau = Math.max(0.015, responseTime - moveDelay);
        double[][] table = buildFeedforwardTable(kS);
        topSpeed = table != null ? table[0][table[0].length - 1] : (maxPower - kS) / fitKV;
        reversalSpeed = Math.max(25, 0.15 * topSpeed);

        baseTune = new TurretTune();
        baseTune.kS = kS;
        baseTune.kV = fitKV;
        baseTune.kA = fitKV * tau;
        baseTune.kVelocityFeedback = 0;
        baseTune.maxCorrection = clamp(1.15 * satPower, 0.06, maxPower);
        baseTune.maxVelocity = 0.75 * topSpeed;
        baseTune.maxAcceleration = clamp(0.35 * topSpeed / tau, 300, 30000);
        // Pure transport delay plus half the rise; the slew-limited rise itself is the profile's job (maxAcceleration).
        baseTune.latency = clamp(moveDelay + 0.5 * tau, 0, 0.3);
        if (table != null) {
            baseTune.ffSpeed = table[0];
            baseTune.ffPower = table[1];
            // Power past the level where speed stops rising (servo output saturates) only wastes feedback range.
        }

        modelKP = 0.5 * fitKV / (tau + moveDelay);
        baseTune.kP = GAIN_START * modelKP;

        searchAngle = Math.min(searchAngle, 0.65 * safeRange);

        plantParams = buildPlantParams(table, kS);
        baseTune.plantDelay = plantParams.delay;
        baseTune.plantAccel = plantParams.accel;
        baseTune.plantDecel = plantParams.decel;
        modelSearch = new TurretModelSearch(plantParams, baseTune, modelKP, searchAngle, topSpeed, safeRange);
        io.setPower(0);
        enter(State.MODEL_SEARCH);
    }

    /** Plant model from what the sweep measured: speed table, start threshold, delay, slew limits, noise, loop time. */
    private TurretPlantModel.Params buildPlantParams(double[][] table, double kS) {
        TurretPlantModel.Params pp = new TurretPlantModel.Params();
        if (table != null) {
            pp.tableSpeed = table[0];
            pp.tablePower = table[1];
        } else {
            pp.tableSpeed = new double[]{0, topSpeed};
            pp.tablePower = new double[]{kS, Math.max(kS + 0.01, satPower)};
        }
        pp.startPower = Math.max(kS, kSRamp);

        double[] fit = fitDelayAndAccel();
        pp.delay = fit[0];
        double a = fit[1];
        pp.accel = a;

        List<Double> decels = new ArrayList<>();
        for (double[] s : decelSamples) {
            double stopDistance = s[1] - s[0] * pp.delay;
            double floor = s[0] * s[0] / (2 * 50000.0);
            decels.add(s[0] * s[0] / (2 * Math.max(floor, stopDistance)));
        }
        pp.decel = decels.isEmpty() ? a : clamp(median(decels), 0.5 * a, 20 * a);

        pp.loopDt = clamp(loopDtAvg, 0.004, 0.06);
        pp.loopJitter = 0.35 * pp.loopDt;
        pp.encoderNoise = noiseN > 20 ? clamp(Math.sqrt(noiseSumSq / noiseN / 2.0), 0.02, 0.5) : 0.08;
        return pp;
    }

    /**
     * Start of every recorded sweep run against 'nothing for the delay, then speed ramps at a to the run's speed'.
     * Grid search for the delay (s) and acceleration (deg/s^2) with the least squared position error.
     */
    private double[] fitDelayAndAccel() {
        double bestErr = Double.POSITIVE_INFINITY, bestDelay = 0.03, bestAccel = 2000;
        for (double delay = 0.0; delay <= 0.15; delay += 0.0025) {
            for (double a = 200; a <= 40000; a *= 1.12) {
                double err = 0;
                int count = 0;
                for (RunTrace run : runTraces) {
                    double ramp = run.speed / a;
                    for (double[] s : run.samples) {
                        if (s[1] > 25) break;   // only the start: after that it is cruising
                        double t = s[0] - delay;
                        double predicted;
                        if (t <= 0) predicted = 0;
                        else if (t < ramp) predicted = 0.5 * a * t * t;
                        else predicted = 0.5 * run.speed * ramp + run.speed * (t - ramp);
                        double d = predicted - s[1];
                        err += d * d;
                        count++;
                    }
                }
                if (count > 0 && err / count < bestErr) {
                    bestErr = err / count;
                    bestDelay = delay;
                    bestAccel = a;
                }
            }
        }
        if (runTraces.isEmpty()) {
            double v = Math.max(50, topSpeed);
            bestDelay = clamp(0.5 * moveDelay, 0.005, 0.2);
            bestAccel = clamp(0.63 * v / Math.max(0.02, responseTime - bestDelay), 200, 40000);
        }
        return new double[]{bestDelay, bestAccel};
    }

    private void modelStep() {
        io.setPower(0);
        boolean finished = modelSearch.run(MODEL_SLICE_NANOS);
        message = String.format("optimising gains on the measured model: generation %d/%d, %d tried",
                Math.min(modelSearch.getGeneration() + 1, modelSearch.getGenerations()),
                modelSearch.getGenerations(), modelSearch.getEvaluations());
        if (!finished) return;

        TurretModelSearch.Scored top = modelSearch.getBest();
        modelBestScore = top == null ? Double.NaN : top.score;
        candidates.clear();
        candidates.addAll(modelSearch.finalists());
        phase = Phase.SEARCH;
        searchRound = 0;
        candidateIndex = 0;
        startCandidate();
    }

    /** Average speed at each sweep power (both directions), as {speeds, powers} starting at (0, kS). */
    private double[][] buildFeedforwardTable(double kS) {
        List<double[]> levels = new ArrayList<>();
        for (RunPlan plan : sweepPlan) {
            if (plan.dir != 1) continue;
            double sum = 0;
            int count = 0;
            for (double[] point : sweepPoints) {
                if (Math.abs(point[0] - plan.power) < 1e-9) {
                    sum += point[1];
                    count++;
                }
            }
            if (count > 0) levels.add(new double[]{sum / count, plan.power});
        }

        List<double[]> rows = new ArrayList<>();
        rows.add(new double[]{0, kS});
        for (double[] level : levels) {
            double[] prev = rows.get(rows.size() - 1);
            if (level[0] > 1.05 * prev[0] + 1 && level[1] > prev[1]) rows.add(level);
        }
        if (rows.size() < 3) return null;

        double[] speed = new double[rows.size()];
        double[] power = new double[rows.size()];
        for (int i = 0; i < rows.size(); i++) {
            speed[i] = rows.get(i)[0];
            power[i] = rows.get(i)[1];
        }
        return new double[][]{speed, power};
    }

    // ---------------------------------------------------------------- trial runner

    private void startTrial(TurretTune gains, double[] targets, boolean track) {
        trialGains = gains;
        trialTargets = targets;
        trialTrack = track;
        trialMoveIndex = 0;
        trialScoreSum = 0;
        trialSettleSum = 0;
        trialOvershootSum = 0;
        trialFailure = null;
        trialStart = now;

        if (Math.abs(angle + searchAngle) > GOTO_TOLERANCE + 1) {
            startGoto(-searchAngle, State.TRIAL_MOVE);
        } else {
            enter(State.TRIAL_MOVE);
        }
    }

    private void startMove(double target) {
        moveTarget = target;
        moveStartTime = now;
        moveStartAngle = angle;
        moveOvershoot = 0;
        settleSince = -1;
        moveStalledSince = -1;
        resetOscillationCheck();
        double distance = Math.abs(target - angle);
        moveTimeout = 1.5 + distance / trialGains.maxVelocity + trialGains.maxVelocity / trialGains.maxAcceleration;
        io.moveTo(target, trialGains);
        moveStarted = true;
    }

    private void resetOscillationCheck() {
        reversals = 0;
        lastVelocitySign = 0;
        errorCrossings = 0;
        lastErrorSign = 0;
        errorPeakSinceCrossing = 0;
    }

    /** Returns a reason if the turret is oscillating or out of range, otherwise null. */
    private String checkOscillation(double target) {
        if (Math.abs(angle) > safeRange) {
            return "left the safe range";
        }

        // Reversals only count away from the target: small corrective moves at the end are not oscillation.
        if (Math.abs(velocity) > reversalSpeed && Math.abs(target - angle) > REVERSAL_MIN_ERROR) {
            double sign = Math.signum(velocity);
            if (lastVelocitySign != 0 && sign != lastVelocitySign) reversals++;
            lastVelocitySign = sign;
        }

        double error = target - angle;
        errorPeakSinceCrossing = Math.max(errorPeakSinceCrossing, Math.abs(error));
        if (Math.abs(error) > 0.3) {
            double sign = Math.signum(error);
            if (lastErrorSign != 0 && sign != lastErrorSign) {
                if (errorPeakSinceCrossing > OSCILLATION_ERROR) errorCrossings++;
                errorPeakSinceCrossing = 0;
            }
            lastErrorSign = sign;
        }

        if (reversals >= OSCILLATION_REVERSALS || errorCrossings >= OSCILLATION_REVERSALS) {
            return "oscillating";
        }
        return null;
    }

    private void trialMove() {
        if (!moveStarted) {
            startMove(trialTargets[trialMoveIndex]);
            return;
        }
        message = trialMessage(String.format("move %d/%d", trialMoveIndex + 1, trialTargets.length));

        String problem = checkOscillation(moveTarget);
        if (problem != null) {
            failTrial(problem);
            return;
        }

        // A candidate that commands nothing (turret parked >10 deg off) is dropped after a second, not after the timeout.
        if (Math.abs(velocity) < 3 && Math.abs(moveTarget - angle) > 10) {
            if (moveStalledSince < 0) moveStalledSince = now;
            if (now - moveStalledSince > 1.0) {
                failTrial("turret not moving");
                return;
            }
        } else {
            moveStalledSince = -1;
        }

        double direction = Math.signum(moveTarget - moveStartAngle);
        moveOvershoot = Math.max(moveOvershoot, (angle - moveTarget) * direction);

        // Settled = on target for SETTLE_HOLD. Position only: the encoder's velocity is noisy at rest and the final
        // approach makes small corrective moves, neither of which is a reason to call a turret that is on target unsettled.
        boolean settled = false;
        if (Math.abs(moveTarget - angle) < SETTLE_TOLERANCE) {
            if (settleSince < 0) settleSince = now;
            settled = now - settleSince >= SETTLE_HOLD;
        } else {
            settleSince = -1;
        }

        double settleTime;
        if (settled) {
            settleTime = settleSince - moveStartTime;
        } else if (now - moveStartTime > moveTimeout) {
            double miss = Math.abs(moveTarget - angle);
            if (miss > SOFT_SETTLE_ERROR) {
                failTrial(String.format("didn't settle (%.1f deg off)", miss));
                return;
            }
            // close but never held inside tolerance: usable, scored as slow
            settleTime = (now - moveStartTime) + 0.3 + 0.2 * miss;
        } else {
            return;
        }

        trialScoreSum += settleTime + 0.1 * moveOvershoot;
        trialSettleSum += settleTime;
        trialOvershootSum += moveOvershoot;
        trialMoveIndex++;
        moveStarted = false;

        if (trialMoveIndex < trialTargets.length) return;

        if (trialTrack) {
            trackStart = now;
            trackSumSq = 0;
            trackCount = 0;
            enter(State.TRIAL_TRACK);
            resetOscillationCheck();
        } else {
            finishTrial(0);
        }
    }

    /** Follows a smooth moving target (like the shot planner does), starting and ending at the last move target. */
    private void trialTrack() {
        double t = now - trackStart;
        message = trialMessage("tracking");

        double base = trialTargets[trialTargets.length - 1];
        double phaseAngle = 2 * Math.PI * Math.min(t, TRACK_SECONDS) / TRACK_SECONDS;
        double target = base + TRACK_AMPLITUDE * (1 - Math.cos(phaseAngle));
        io.moveTo(target, trialGains);

        if (Math.abs(angle) > safeRange) {
            failTrial("left the safe range");
            return;
        }
        if (Math.abs(velocity) > reversalSpeed) {
            double sign = Math.signum(velocity);
            if (lastVelocitySign != 0 && sign != lastVelocitySign) reversals++;
            lastVelocitySign = sign;
        }
        if (reversals >= OSCILLATION_REVERSALS || Math.abs(target - angle) > 3 * TRACK_AMPLITUDE / 5 + 5) {
            failTrial("oscillating while tracking");
            return;
        }

        if (t > 0.3) {
            trackSumSq += (target - angle) * (target - angle);
            trackCount++;
        }
        if (t < TRACK_SECONDS + 0.4) return;

        finishTrial(trackCount > 0 ? Math.sqrt(trackSumSq / trackCount) : 0);
    }

    private void failTrial(String reason) {
        io.setPower(0);
        trialFailure = reason;
        enter(State.BRAKE);
    }

    private void brake() {
        io.setPower(0);
        message = trialMessage("stopped: " + trialFailure);
        if (!isStill(0.25) && elapsed() < 2.5) return;
        onTrialDone(false, 0);
    }

    private void finishTrial(double trackingRms) {
        onTrialDone(true, trackingRms);
    }

    private String trialMessage(String detail) {
        switch (phase) {
            case SEARCH:
                return String.format("%s %d/%d on the turret: %s",
                        searchRound == 0 ? "checking model candidate" : "fallback candidate",
                        candidateIndex + 1, candidates.size(), detail);
            default:
                return String.format("verify attempt %d/%d: %s", verifyAttempt + 1, VERIFY_ATTEMPTS, detail);
        }
    }

    private void onTrialDone(boolean passed, double trackingRms) {
        switch (phase) {
            case SEARCH: searchTrialDone(passed, trackingRms); break;
            case VERIFY: verifyTrialDone(passed); break;
        }
    }

    // ---------------------------------------------------------------- candidates on the turret

    /** Round 0 is built by the model search; later rounds are fallbacks that only run if nothing was stable. */
    private void buildFallbackRound() {
        candidates.clear();
        candidateIndex = 0;
        double factor = searchRound == 1 ? 0.5 : 0.25;
        TurretTune t = baseTune.copy();
        t.kP = modelKP * factor;
        t.kVelocityFeedback = 0.25 * baseTune.kV;
        t.brakeAware = false;
        t.maxAcceleration = Math.max(300, baseTune.maxAcceleration * factor);
        t.maxVelocity = baseTune.maxVelocity * (searchRound == 1 ? 0.8 : 0.6);
        candidates.add(t);
    }

    private void startCandidate() {
        startTrial(candidates.get(candidateIndex), new double[]{searchAngle, -searchAngle}, true);
    }

    private void searchTrialDone(boolean passed, double trackingRms) {
        candidatesTried++;
        if (passed) {
            int moves = trialTargets.length;
            double score = trialScoreSum / moves + TRACK_WEIGHT * trackingRms;
            if (score < bestScore) {
                bestScore = score;
                best = trialGains.copy();
                bestSettle = trialSettleSum / moves;
                bestOvershoot = trialOvershootSum / moves;
                bestTracking = trackingRms;
            }
        } else {
            candidatesFailed++;
        }

        candidateIndex++;
        if (candidateIndex < candidates.size()) {
            startCandidate();
            return;
        }

        if (best == null) {
            searchRound++;
            if (searchRound < SEARCH_ROUNDS) {
                warning = "model's settings were not stable on the turret, trying gentler gains";
                buildFallbackRound();
                startCandidate();
                return;
            }
            // Nothing passed even gently: keep the gentlest settings rather than throw the sweep away.
            result = candidates.get(candidates.size() - 1).copy();
            verified = false;
            warning = "UNVERIFIED: no setting passed the on-turret check (" + trialFailure + "). Gentle gains saved.";
            io.moveTo(0, result);
            enter(State.FINISH_MOVE);
            return;
        }

        phase = Phase.VERIFY;
        verifyAttempt = 0;
        startVerify();
    }

    // ---------------------------------------------------------------- verify

    private void startVerify() {
        startTrial(best.copy(), new double[]{searchAngle, -searchAngle, -searchAngle + GAIN_STEP, -searchAngle}, true);
    }

    private void verifyTrialDone(boolean passed) {
        if (passed) {
            result = trialGains.copy();
            verified = true;
            io.moveTo(0, result);
            enter(State.FINISH_MOVE);
            return;
        }

        verifyAttempt++;
        if (verifyAttempt >= VERIFY_ATTEMPTS) {
            // It passed the search trial, so keep it (already backed off) and say it did not pass the longer check.
            result = best.copy();
            verified = false;
            warning = "UNVERIFIED: passed the first check but not the long one (" + trialFailure + "). Saved with reduced gains.";
            io.moveTo(0, result);
            enter(State.FINISH_MOVE);
            return;
        }
        best.kP *= VERIFY_BACKOFF;
        best.kVelocityFeedback *= VERIFY_BACKOFF;
        best.maxAcceleration = Math.max(300, best.maxAcceleration * VERIFY_BACKOFF);
        warning = String.format("final check failed (%s), backing off gains", trialFailure);
        startVerify();
    }

    private void finishMove() {
        message = "returning to 0";
        if ((Math.abs(angle) < SETTLE_TOLERANCE && Math.abs(velocity) < SETTLE_VELOCITY) || elapsed() > 3) {
            message = "done";
            enter(State.DONE);
        }
    }

    // ---------------------------------------------------------------- results

    public boolean isDone() {
        return state == State.DONE;
    }

    public boolean isAborted() {
        return state == State.ABORTED;
    }

    public Stage getStage() {
        switch (state) {
            case KS_WAIT_STILL:
            case KS_RAMP:
                return Stage.DEADBAND;
            case DONE:
                return Stage.DONE;
            case ABORTED:
                return Stage.ABORTED;
            case FINISH_MOVE:
                return Stage.VERIFY;
            case MODEL_SEARCH:
                return Stage.MODEL;
            default:
                if (baseTune == null) return Stage.SWEEP;
                if (phase == Phase.GAIN_LIMIT) return Stage.GAIN_LIMIT;
                return phase == Phase.SEARCH ? Stage.SEARCH : Stage.VERIFY;
        }
    }

    public String getMessage() { return message; }
    public String getWarning() { return warning; }
    public TurretTune getResult() { return result == null ? null : result.copy(); }
    public TurretTune getBestSoFar() { return best == null ? null : best.copy(); }
    public TurretTune getBaseTune() { return baseTune == null ? null : baseTune.copy(); }
    public double getDeadbandRamp() { return kSRamp; }
    public double getFitKS() { return fitKS; }
    public double getFitKV() { return fitKV; }
    public double getFitR2() { return fitR2; }
    public double getKVPositive() { return kVPositive; }
    public double getKVNegative() { return kVNegative; }
    public double getTopSpeed() { return topSpeed; }
    public double getResponseTime() { return responseTime; }
    public double getMoveDelay() { return moveDelay; }
    public double getGainLimitKP() { return gainLimitKP; }
    public boolean isGainLimitFound() { return gainLimitFound; }
    public boolean isVerified() { return verified; }
    public double getModelBestScore() { return modelBestScore; }
    public TurretPlantModel.Params getPlantParams() { return plantParams; }
    public double getBestSettle() { return bestSettle; }
    public double getBestOvershoot() { return bestOvershoot; }
    public double getBestTracking() { return bestTracking; }
    public int getCandidatesTried() { return candidatesTried; }
    public int getCandidatesFailed() { return candidatesFailed; }
    public int getSweepPointCount() { return sweepPoints.size(); }

    public List<double[]> getSweepPoints() {
        return new ArrayList<>(sweepPoints);
    }

    // ---------------------------------------------------------------- helpers

    private void enter(State next) {
        state = next;
        stateStart = now;
        stillSince = -1;
        moveStarted = false;
    }

    private double elapsed() {
        return now - stateStart;
    }

    private boolean isStill(double holdSeconds) {
        if (Math.abs(velocity) < 5) {
            if (stillSince < 0) stillSince = now;
            return now - stillSince >= holdSeconds;
        }
        stillSince = -1;
        return false;
    }

    private void abort(String reason) {
        io.setPower(0);
        message = "ABORTED: " + reason;
        state = State.ABORTED;
    }

    /** Speed at the two highest sweep powers measured so far is no longer rising: stop adding levels. */
    private boolean saturationReached() {
        double lastPower = -1, prevPower = -1;
        double lastSum = 0, prevSum = 0;
        int lastN = 0, prevN = 0;
        for (double[] p : sweepPoints) {
            if (p[0] > lastPower + 1e-9) {
                prevPower = lastPower; prevSum = lastSum; prevN = lastN;
                lastPower = p[0]; lastSum = 0; lastN = 0;
            }
            lastSum += p[1];
            lastN++;
        }
        if (prevN == 0 || lastN == 0) return false;
        double last = lastSum / lastN, prev = prevSum / prevN;
        return last <= (1 + SATURATION_STEP) * prev;
    }

    /** Median of (power - kS) / speed over one direction's points: robust to the stiction-affected low runs. */
    private static double directionKV(List<double[]> points, int dir, double kS) {
        List<Double> ratios = new ArrayList<>();
        for (double[] p : points) {
            if (p[2] == dir && p[1] > 0) ratios.add(Math.max(0, p[0] - kS) / p[1]);
        }
        return ratios.isEmpty() ? Double.NaN : median(ratios);
    }

    private static int countDirection(List<double[]> points, int dir) {
        int n = 0;
        for (double[] p : points) if (p[2] == dir) n++;
        return n;
    }

    /** Median of slopes between sample pairs at least 3 samples apart. columns: x=time, y=angle. */
    private static double robustSlope(List<double[]> rows) {
        List<Double> slopes = new ArrayList<>();
        for (int i = 0; i < rows.size(); i++) {
            for (int j = i + 3; j < rows.size(); j++) {
                double dt = rows.get(j)[0] - rows.get(i)[0];
                if (dt > 1e-6) slopes.add((rows.get(j)[1] - rows.get(i)[1]) / dt);
            }
        }
        return slopes.isEmpty() ? Double.NaN : median(slopes);
    }

    /** Least squares y = a + b x over columns xi, yi. Returns {a, b, r2}. */
    private static double[] lineFit(List<double[]> rows, int xi, int yi) {
        int n = rows.size();
        double sx = 0, sy = 0;
        for (double[] r : rows) {
            sx += r[xi];
            sy += r[yi];
        }
        double mx = sx / n, my = sy / n;
        double sxx = 0, sxy = 0, syy = 0;
        for (double[] r : rows) {
            double dx = r[xi] - mx, dy = r[yi] - my;
            sxx += dx * dx;
            sxy += dx * dy;
            syy += dy * dy;
        }
        double b = sxx > 0 ? sxy / sxx : Double.NaN;
        double a = my - b * mx;
        double r2 = (sxx > 0 && syy > 0) ? (sxy * sxy) / (sxx * syy) : 0;
        return new double[]{a, b, r2};
    }

    /** Fits power = kS + kV * speed. dir 0 uses both directions. */
    private static double[] fitPowerVsSpeed(List<double[]> points, int dir) {
        List<double[]> rows = new ArrayList<>();
        for (double[] p : points) {
            if (dir == 0 || p[2] == dir) rows.add(new double[]{p[1], p[0]});
        }
        if (rows.size() < 2) return new double[]{Double.NaN, Double.NaN, 0};
        return lineFit(rows, 0, 1);
    }

    private static double mean(List<Double> values) {
        double sum = 0;
        for (double v : values) sum += v;
        return sum / values.size();
    }

    private static double median(List<Double> values) {
        double[] sorted = new double[values.size()];
        for (int i = 0; i < sorted.length; i++) sorted[i] = values.get(i);
        Arrays.sort(sorted);
        int mid = sorted.length / 2;
        return sorted.length % 2 == 1 ? sorted[mid] : 0.5 * (sorted[mid - 1] + sorted[mid]);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
