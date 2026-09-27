package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Measures the turret and picks controller gains. Call step() once per loop.
 *
 * 1. Deadband: ramp power slowly until the turret starts to move (kS).
 * 2. Sweep: full-travel runs at several powers, both directions. Steady speed vs power gives kV,
 *    and how fast each run gets up to speed gives the response time (kA, max acceleration).
 * 3. Search: closed-loop moves with candidate kP / velocity feedback / acceleration, scored on
 *    settle time and overshoot. The best one wins.
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

    public enum Stage { DEADBAND, SWEEP, SEARCH, DONE, ABORTED }

    private enum State {
        KS_WAIT_STILL, KS_RAMP,
        GOTO, SWEEP_REST, SWEEP_RUN,
        SEARCH_MOVE, FINISH_MOVE,
        DONE, ABORTED
    }

    public double safeRange = 110;
    public double abortMargin = 25;
    public double maxPower = 0.5;
    public double searchAngle = 70;

    private static final double KS_RAMP_RATE = 0.03;
    private static final double KS_RAMP_LIMIT = 0.3;
    private static final int KS_RAMPS = 4;
    private static final double[] SWEEP_FRACTIONS = {0.2, 0.4, 0.6, 0.8, 1.0};
    private static final double RUN_TIMEOUT = 6.0;
    private static final double SETTLE_TOLERANCE = 1.0;
    private static final double SETTLE_VELOCITY = 15;
    private static final double SETTLE_HOLD = 0.15;
    private static final int SEARCH_ROUNDS = 4;
    private static final double TRACK_SECONDS = 2.0;
    private static final double TRACK_AMPLITUDE = 25.0;
    private static final double TRACK_WEIGHT = 0.1;

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

    private TurretTune baseTune;
    private final List<TurretTune> candidates = new ArrayList<>();
    private int searchRound = 0;
    private int candidateIndex = 0;
    private int moveInCandidate = 0;
    private double candidateScoreSum = 0;
    private TurretTune best;
    private double bestScore = Double.POSITIVE_INFINITY;
    private double bestSettle, bestOvershoot, bestTracking;
    private double candidateSettleSum, candidateOvershootSum;
    private int candidatesTried = 0;

    private double moveTarget;
    private double moveStartTime;
    private double moveStartAngle;
    private double moveOvershoot;
    private double settleSince = -1;
    private double moveTimeout;

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
            case SEARCH_MOVE: searchMove(); break;
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
        gotoTarget = target;
        afterGoto = next;
        haveCut = false;
        enter(State.GOTO);
    }

    private void gotoStep() {
        double error = gotoTarget - angle;
        message = String.format("moving to %.0f deg (at %.1f)", gotoTarget, angle);

        if (Math.abs(error) < 6) {
            io.setPower(0);
            enter(afterGoto);
            return;
        }
        if (elapsed() > 8) {
            abort("couldn't reach " + gotoTarget + " deg");
            return;
        }

        double power = kSRamp + 0.02 + Math.min(0.1, 0.003 * Math.abs(error));
        io.setPower(Math.signum(error) * Math.min(maxPower, power));
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

        double kS = clamp(fitKS, 0.7 * kSRamp, kSRamp);

        responseTime = t63Samples.isEmpty() ? 0.08 : median(t63Samples);
        moveDelay = moveDelaySamples.isEmpty() ? 0.03 : median(moveDelaySamples);
        tau = Math.max(0.015, responseTime - moveDelay);
        topSpeed = (maxPower - kS) / fitKV;

        baseTune = new TurretTune();
        baseTune.kS = kS;
        baseTune.kV = fitKV;
        baseTune.kA = fitKV * tau;
        baseTune.kP = 0.5 * fitKV / (tau + moveDelay);
        baseTune.kVelocityFeedback = 0;
        baseTune.maxCorrection = maxPower;
        baseTune.maxVelocity = 0.75 * topSpeed;
        baseTune.maxAcceleration = clamp(0.35 * topSpeed / tau, 300, 30000);
        baseTune.latency = clamp(responseTime, 0, 0.3);

        searchAngle = Math.min(searchAngle, 0.65 * safeRange);
        searchRound = 0;
        buildSearchRound();
        startGoto(-searchAngle, State.SEARCH_MOVE);
        pendingPrepMove = true;
    }

    // ---------------------------------------------------------------- search

    private boolean pendingPrepMove = false;

    private void buildSearchRound() {
        candidates.clear();
        candidateIndex = 0;
        moveInCandidate = 0;
        candidateScoreSum = 0;
        candidateSettleSum = 0;
        candidateOvershootSum = 0;

        if (searchRound == 0) {
            for (double vf : new double[]{0, 0.5}) {
                for (double m : new double[]{0.5, 1.0, 1.5, 2.5}) {
                    TurretTune t = baseTune.copy();
                    t.kP = baseTune.kP * m;
                    t.kVelocityFeedback = baseTune.kV * vf;
                    candidates.add(t);
                }
            }
        } else if (searchRound == 1) {
            for (double m : new double[]{0.5, 0.75, 1.25, 1.6}) {
                TurretTune t = best.copy();
                t.latency = clamp(best.latency * m, 0, 0.3);
                candidates.add(t);
            }
        } else if (searchRound == 2) {
            for (double m : new double[]{0.6, 1.6, 2.5}) {
                TurretTune t = best.copy();
                t.maxAcceleration = clamp(best.maxAcceleration * m, 300, 30000);
                candidates.add(t);
            }
            for (double m : new double[]{0.0, 0.5}) {
                TurretTune t = best.copy();
                t.kA = best.kA * m;
                candidates.add(t);
            }
        } else {
            for (double m : new double[]{0.8, 1.25}) {
                TurretTune t = best.copy();
                t.kP = best.kP * m;
                candidates.add(t);
            }
            for (double vf : new double[]{0.25, 1.0}) {
                TurretTune t = best.copy();
                t.kVelocityFeedback = best.kV * vf;
                candidates.add(t);
            }
            for (double m : new double[]{0.8, 1.2}) {
                TurretTune t = best.copy();
                t.kS = best.kS * m;
                candidates.add(t);
            }
        }
    }

    private void startMove(double target) {
        TurretTune gains = pendingPrepMove ? baseTune : candidates.get(candidateIndex);
        moveTarget = target;
        moveStartTime = now;
        moveStartAngle = angle;
        moveOvershoot = 0;
        settleSince = -1;
        double distance = Math.abs(target - angle);
        moveTimeout = 1.5 + distance / gains.maxVelocity + gains.maxVelocity / gains.maxAcceleration;
        io.moveTo(target, gains);
    }

    private boolean moveStarted = false;
    private boolean tracking = false;
    private double trackStart;
    private double trackSumSq;
    private int trackCount;

    private void searchMove() {
        if (tracking) {
            trackStep();
            return;
        }

        if (!moveStarted) {
            double target = pendingPrepMove ? -searchAngle
                    : (moveInCandidate == 0 ? searchAngle : -searchAngle);
            startMove(target);
            moveStarted = true;
            return;
        }

        message = pendingPrepMove
                ? "search: moving to start"
                : String.format("search round %d: candidate %d/%d, move %d",
                        searchRound + 1, candidateIndex + 1, candidates.size(), moveInCandidate + 1);

        double direction = Math.signum(moveTarget - moveStartAngle);
        moveOvershoot = Math.max(moveOvershoot, (angle - moveTarget) * direction);

        boolean settled = false;
        if (Math.abs(moveTarget - angle) < SETTLE_TOLERANCE && Math.abs(velocity) < SETTLE_VELOCITY) {
            if (settleSince < 0) settleSince = now;
            settled = now - settleSince >= SETTLE_HOLD;
        } else {
            settleSince = -1;
        }
        boolean timedOut = now - moveStartTime > moveTimeout;

        if (!settled && !timedOut) return;
        moveStarted = false;

        if (pendingPrepMove) {
            pendingPrepMove = false;
            if (!settled) {
                startGoto(-searchAngle, State.SEARCH_MOVE);
            }
            return;
        }

        double settleTime = settled ? settleSince - moveStartTime : moveTimeout;
        double score = settled
                ? settleTime + 0.1 * moveOvershoot
                : 10 + Math.abs(moveTarget - angle);

        candidateScoreSum += score;
        candidateSettleSum += settleTime;
        candidateOvershootSum += moveOvershoot;
        moveInCandidate++;

        if (!settled) {
            finishCandidate(false, 0);
            return;
        }
        if (moveInCandidate >= 2) {
            tracking = true;
            trackStart = now;
            trackSumSq = 0;
            trackCount = 0;
        }
    }

    /** Follows a smooth moving target (like the shot planner does) starting and ending at -searchAngle. */
    private void trackStep() {
        double t = now - trackStart;
        message = String.format("search round %d: candidate %d/%d, tracking",
                searchRound + 1, candidateIndex + 1, candidates.size());

        double phase = 2 * Math.PI * Math.min(t, TRACK_SECONDS) / TRACK_SECONDS;
        double target = -searchAngle + TRACK_AMPLITUDE * (1 - Math.cos(phase));
        io.moveTo(target, candidates.get(candidateIndex));

        if (t > 0.3) {
            trackSumSq += (target - angle) * (target - angle);
            trackCount++;
        }
        if (t < TRACK_SECONDS + 0.4) return;

        tracking = false;
        double rms = trackCount > 0 ? Math.sqrt(trackSumSq / trackCount) : 0;
        finishCandidate(true, rms);
    }

    private void finishCandidate(boolean completed, double trackingRms) {
        int moves = moveInCandidate;
        double candidateScore = completed
                ? candidateScoreSum / moves + TRACK_WEIGHT * trackingRms
                : 10 + candidateScoreSum;
        candidatesTried++;
        if (candidateScore < bestScore) {
            bestScore = candidateScore;
            best = candidates.get(candidateIndex).copy();
            bestSettle = candidateSettleSum / moves;
            bestOvershoot = candidateOvershootSum / moves;
            bestTracking = trackingRms;
        }

        candidateIndex++;
        moveInCandidate = 0;
        candidateScoreSum = 0;
        candidateSettleSum = 0;
        candidateOvershootSum = 0;
        moveStarted = false;

        boolean atStart = completed && Math.abs(angle + searchAngle) < 3 && Math.abs(velocity) < SETTLE_VELOCITY;

        if (candidateIndex >= candidates.size()) {
            if (best == null) {
                abort("no candidate settled - turret may be binding");
                return;
            }
            searchRound++;
            if (searchRound >= SEARCH_ROUNDS) {
                io.moveTo(0, best);
                enter(State.FINISH_MOVE);
                return;
            }
            buildSearchRound();
        }

        if (!atStart) {
            startGoto(-searchAngle, State.SEARCH_MOVE);
        }
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
            case SEARCH_MOVE:
            case FINISH_MOVE:
                return Stage.SEARCH;
            case DONE:
                return Stage.DONE;
            case ABORTED:
                return Stage.ABORTED;
            default:
                return baseTune == null ? Stage.SWEEP : Stage.SEARCH;
        }
    }

    public String getMessage() { return message; }
    public String getWarning() { return warning; }
    public TurretTune getResult() { return best == null ? null : best.copy(); }
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
    public double getBestSettle() { return bestSettle; }
    public double getBestOvershoot() { return bestOvershoot; }
    public double getBestTracking() { return bestTracking; }
    public int getCandidatesTried() { return candidatesTried; }
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
        tracking = false;
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
