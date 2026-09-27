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

    public enum Stage { DEADBAND, SWEEP, GAIN_LIMIT, SEARCH, VERIFY, DONE, ABORTED }

    private enum State {
        KS_WAIT_STILL, KS_RAMP,
        GOTO, SWEEP_REST, SWEEP_RUN,
        TRIAL_MOVE, TRIAL_TRACK, BRAKE,
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
    private static final double[] SWEEP_FRACTIONS = {0.05, 0.12, 0.25, 0.45, 0.7, 1.0};
    private static final double RUN_TIMEOUT = 6.0;
    private static final double GOTO_TOLERANCE = 5.0;
    private static final double SETTLE_TOLERANCE = 1.0;
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
    private static final double VERIFY_BACKOFF = 0.7;
    private static final double[][] SEARCH_KP_OF_KU = {{0.2, 0.3, 0.45}, {0.8, 1.2}};

    private final Io io;

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

    private final List<double[]> sweepPoints = new ArrayList<>();
    private final List<Double> t63Samples = new ArrayList<>();
    private final List<Double> moveDelaySamples = new ArrayList<>();

    private double fitKS, fitKV, fitR2, kVPositive, kVNegative;
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
        enter(State.GOTO);
    }

    /** Open-loop move: drive, cut power early enough to coast in, retry gentler if it misses. */
    private void gotoStep() {
        double error = gotoTarget - angle;
        message = String.format("moving to %.0f deg (at %.1f)", gotoTarget, angle);

        if (elapsed() > 15) {
            abort("couldn't reach " + gotoTarget + " deg");
            return;
        }

        if (gotoMoving) {
            if (Math.abs(error) < 3 + coastFactor * Math.abs(velocity)) {
                io.setPower(0);
                gotoMoving = false;
                stillSince = -1;
                return;
            }
            double scale = Math.pow(0.5, gotoAttempts);
            double power = kSRamp + scale * (0.02 + Math.min(0.1, 0.003 * Math.abs(error)));
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
        }
        haveCut = false;

        if (runIndex >= sweepPlan.size()) {
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
                fit = lineFit(tail, 0, 1);
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

        double[] fit = fitPowerVsSpeed(sweepPoints, 0);
        fitKS = fit[0];
        fitKV = fit[1];
        fitR2 = fit[2];
        kVPositive = fitPowerVsSpeed(sweepPoints, 1)[1];
        kVNegative = fitPowerVsSpeed(sweepPoints, -1)[1];

        if (!(fitKV > 0)) {
            abort("speed didn't increase with power - check the turret isn't jammed");
            return;
        }
        if (fitR2 < 0.9) {
            warning = String.format("speed vs power isn't very linear (R2 %.2f)", fitR2);
        }
        if (Math.abs(kVPositive - kVNegative) > 0.25 * fitKV) {
            warning = "one direction is noticeably weaker than the other";
        }

        double kS = clamp(fitKS, 0.85 * kSRamp, kSRamp);

        responseTime = t63Samples.isEmpty() ? 0.08 : median(t63Samples);
        moveDelay = moveDelaySamples.isEmpty() ? 0.03 : median(moveDelaySamples);
        tau = Math.max(0.015, responseTime - moveDelay);
        double[][] table = buildFeedforwardTable(kS);
        topSpeed = table != null ? table[0][table[0].length - 1] : (maxPower - kS) / fitKV;
        reversalSpeed = Math.max(20, 0.08 * topSpeed);

        baseTune = new TurretTune();
        baseTune.kS = kS;
        baseTune.kV = fitKV;
        baseTune.kA = fitKV * tau;
        baseTune.kVelocityFeedback = 0;
        baseTune.maxCorrection = maxPower;
        baseTune.maxVelocity = 0.75 * topSpeed;
        baseTune.maxAcceleration = clamp(0.35 * topSpeed / tau, 300, 30000);
        baseTune.latency = clamp(responseTime, 0, 0.3);
        if (table != null) {
            baseTune.ffSpeed = table[0];
            baseTune.ffPower = table[1];
        }

        modelKP = 0.5 * fitKV / (tau + moveDelay);
        baseTune.kP = GAIN_START * modelKP;

        searchAngle = Math.min(searchAngle, 0.65 * safeRange);
        phase = Phase.GAIN_LIMIT;
        startTrial(baseTune.copy(), new double[]{-searchAngle + GAIN_STEP, -searchAngle}, false);
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
            if (level[0] > prev[0] && level[1] > prev[1]) rows.add(level);
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

        if (Math.abs(velocity) > reversalSpeed) {
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

        double direction = Math.signum(moveTarget - moveStartAngle);
        moveOvershoot = Math.max(moveOvershoot, (angle - moveTarget) * direction);

        boolean settled = false;
        if (Math.abs(moveTarget - angle) < SETTLE_TOLERANCE && Math.abs(velocity) < SETTLE_VELOCITY) {
            if (settleSince < 0) settleSince = now;
            settled = now - settleSince >= SETTLE_HOLD;
        } else {
            settleSince = -1;
        }

        if (!settled) {
            if (now - moveStartTime > moveTimeout) {
                failTrial("didn't settle");
            }
            return;
        }

        double settleTime = settleSince - moveStartTime;
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
            case GAIN_LIMIT:
                return String.format("gain limit: kP %.5f, %s", trialGains.kP, detail);
            case SEARCH:
                return String.format("search round %d, candidate %d/%d: %s",
                        searchRound + 1, candidateIndex + 1, candidates.size(), detail);
            default:
                return String.format("verify attempt %d/%d: %s", verifyAttempt + 1, VERIFY_ATTEMPTS, detail);
        }
    }

    private void onTrialDone(boolean passed, double trackingRms) {
        switch (phase) {
            case GAIN_LIMIT: gainLimitTrialDone(passed); break;
            case SEARCH: searchTrialDone(passed, trackingRms); break;
            case VERIFY: verifyTrialDone(passed); break;
        }
    }

    // ---------------------------------------------------------------- gain limit

    private void gainLimitTrialDone(boolean passed) {
        boolean oscillated = !passed && !"didn't settle".equals(trialFailure);
        double kP = trialGains.kP;

        if (oscillated) {
            gainLimitKP = kP;
            gainLimitFound = true;
            startSearch();
            return;
        }

        double next = kP * GAIN_GROWTH;
        if (next > GAIN_MAX * modelKP) {
            gainLimitKP = kP * GAIN_GROWTH;
            gainLimitFound = false;
            startSearch();
            return;
        }

        TurretTune t = baseTune.copy();
        t.kP = next;
        startTrial(t, new double[]{-searchAngle + GAIN_STEP, -searchAngle}, false);
    }

    // ---------------------------------------------------------------- search

    private void startSearch() {
        phase = Phase.SEARCH;
        searchRound = 0;
        buildSearchRound();
        startCandidate();
    }

    private void buildSearchRound() {
        candidates.clear();
        candidateIndex = 0;

        if (searchRound == 0) {
            for (double vf : new double[]{0, 0.25}) {
                for (double m : SEARCH_KP_OF_KU[0]) {
                    TurretTune t = baseTune.copy();
                    t.kP = gainLimitKP * m;
                    t.kVelocityFeedback = baseTune.kV * vf;
                    candidates.add(t);
                }
            }
        } else if (searchRound == 1) {
            for (double m : new double[]{0.5, 1.5}) {
                TurretTune t = best.copy();
                t.latency = clamp(best.latency * m, 0, 0.3);
                candidates.add(t);
            }
        } else if (searchRound == 2) {
            for (double m : new double[]{0.6, 1.6}) {
                TurretTune t = best.copy();
                t.maxAcceleration = clamp(best.maxAcceleration * m, 300, 30000);
                candidates.add(t);
            }
            TurretTune noKA = best.copy();
            noKA.kA = 0;
            candidates.add(noKA);
        } else {
            for (double m : SEARCH_KP_OF_KU[1]) {
                TurretTune t = best.copy();
                t.kP = Math.min(best.kP * m, 0.5 * gainLimitKP);
                if (t.kP != best.kP) candidates.add(t);
            }
            for (double m : new double[]{0.8, 1.2}) {
                TurretTune t = best.copy();
                t.kS = best.kS * m;
                candidates.add(t);
            }
        }
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
            abort("no candidate was stable - turret may be binding or the encoder is noisy");
            return;
        }

        searchRound++;
        if (searchRound < 4) {
            buildSearchRound();
            if (!candidates.isEmpty()) {
                startCandidate();
                return;
            }
            searchRound++;
            if (searchRound < 4) {
                buildSearchRound();
                startCandidate();
                return;
            }
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
            io.moveTo(0, result);
            enter(State.FINISH_MOVE);
            return;
        }

        verifyAttempt++;
        if (verifyAttempt >= VERIFY_ATTEMPTS) {
            abort("couldn't find gains that stay stable (" + trialFailure + ")");
            return;
        }
        best.kP *= VERIFY_BACKOFF;
        best.kVelocityFeedback *= VERIFY_BACKOFF;
        warning = String.format("final check failed (%s), backing off kP", trialFailure);
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
