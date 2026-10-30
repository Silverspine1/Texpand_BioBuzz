package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Random;

/**
 * Picks controller gains by running the real {@link SetTurretAngle} against a {@link TurretPlantModel} of this turret,
 * in simulated time, with a cross-entropy search. No turret motion is needed, so it can try hundreds of settings in
 * seconds; the tuner then checks the best few on the real turret. Call {@link #run} every loop until it returns true.
 */
public class TurretModelSearch {

    public static final class Scored {
        public final TurretTune tune;
        public final double score;
        public final double settle;
        public final double overshoot;
        public final double tracking;
        public final double fastTracking;
        public final double finalError;
        public final boolean failed;

        Scored(TurretTune tune, double score, double settle, double overshoot, double tracking, double fastTracking, double finalError, boolean failed) {
            this.tune = tune;
            this.score = score;
            this.settle = settle;
            this.overshoot = overshoot;
            this.tracking = tracking;
            this.fastTracking = fastTracking;
            this.finalError = finalError;
            this.failed = failed;
        }
    }

    public static final double SETTLE_TOLERANCE = 1.25;
    public static final double SETTLE_HOLD = 0.15;
    public static final double TRACK_SECONDS = 2.0;
    public static final double TRACK_AMPLITUDE = 25.0;
    public static final double TRACK_WEIGHT = 0.1;
    public static final double OVERSHOOT_WEIGHT = 0.1;
    public static final double FAST_AMPLITUDE = 30.0;
    public static final double FAST_SECONDS = 1.4;
    public static final double FAST_TRACK_WEIGHT = 0.25;
    static final double FAIL_SCORE = 50.0;

    static int POPULATION = 70;
    static int ELITES = 14;
    static int GENERATIONS = 20;
    private static final int DIM = 12;

    // parameter vector: ln kP, velocity-feedback / kV, latency s, ln maxAcc, maxVel fraction, kA scale, P(predictive mode),
    // predictive delay scale, kS scale, ln kPos, stopping margin, plant accel scale
    private final double[] LO = {Math.log(2e-4), 0.0, 0.0, Math.log(300), 0.40, 0.0, 0.0, 0.6, 0.8, Math.log(3), 0.4, 0.5};
    private final double[] HI = {Math.log(0.05), 3.0, 0.15, Math.log(12000), 0.97, 3.0, 1.0, 1.8, 1.3, Math.log(80), 1.0, 1.5};

    private final TurretPlantModel.Params plant;
    private final TurretTune base;
    private final double searchAngle;
    private final double topSpeed;
    private final double reversalSpeed;
    private final double safeRange;
    private final Random rng = new Random(12345);

    private final double[] mean = new double[DIM];
    private final double[] sigma = new double[DIM];
    private final List<Scored> generation = new ArrayList<>();
    private final List<Scored> all = new ArrayList<>();
    private int generationIndex = 0;
    private int sampled = 0;
    private double[][] currentVectors = new double[POPULATION][];
    private boolean done = false;
    private int evaluations = 0;
    private Scored best;

    public TurretModelSearch(TurretPlantModel.Params plant, TurretTune base, double modelKP, double searchAngle,
                             double topSpeed, double safeRange) {
        this.plant = plant;
        this.base = base;
        this.searchAngle = searchAngle;
        this.topSpeed = topSpeed;
        this.reversalSpeed = Math.max(25, 0.15 * topSpeed);
        this.safeRange = safeRange;

        mean[0] = Math.log(Math.max(2e-4, modelKP));
        mean[1] = 0.3;
        mean[2] = clamp(base.latency, 0.0, 0.15);
        mean[3] = Math.log(clamp(base.maxAcceleration, 300, 12000));
        mean[4] = 0.8;
        mean[5] = 0.5;
        mean[7] = 1.0;
        mean[8] = 1.0;
        mean[9] = Math.log(analyticKPos());
        LO[9] = Math.log(analyticKPos() / 2.5);
        HI[9] = Math.log(analyticKPos() * 2.5);
        LO[4] = 0.7;
        mean[6] = 0.8;
        mean[10] = 0.8;
        mean[11] = 1.0;
        for (int i = 0; i < DIM; i++) {
            sigma[i] = 0.35 * (HI[i] - LO[i]);
        }
    }

    public int getEvaluations() { return evaluations; }

    public int getGeneration() { return generationIndex; }

    public int getGenerations() { return GENERATIONS; }

    public Scored getBest() { return best; }

    List<Scored> all() { return all; }

    public boolean isDone() { return done; }

    /** Evaluates candidates until the time budget is used. Returns true once the search has finished. */
    public boolean run(long budgetNanos) {
        long end = System.nanoTime() + budgetNanos;
        while (!done && System.nanoTime() < end) {
            if (sampled == 0) {
                for (int i = 0; i < POPULATION; i++) {
                    currentVectors[i] = sample(generationIndex == 0 && i == 0);
                }
                generation.clear();
            }
            Scored s = evaluate(toTune(currentVectors[sampled]));
            generation.add(s);
            all.add(s);
            evaluations++;
            if (best == null || s.score < best.score) best = s;
            sampled++;

            if (sampled == POPULATION) {
                sampled = 0;
                updateDistribution();
                generationIndex++;
                if (generationIndex >= GENERATIONS) done = true;
            }
        }
        return done;
    }

    /** Best few distinct settings: overall, best profiled, best brake-aware, and a gentler version of the winner. */
    public List<TurretTune> finalists() {
        List<Scored> ok = new ArrayList<>();
        for (Scored s : all) if (!s.failed) ok.add(s);
        Collections.sort(ok, BY_SCORE);

        List<TurretTune> out = new ArrayList<>();
        if (ok.isEmpty()) {
            out.add(analytic());
            out.add(gentler(base.copy(), 0.5));
            return out;
        }
        out.add(ok.get(0).tune);
        out.add(analytic());
        for (Scored s : ok) {
            if (!s.tune.brakeAware) { addIfNew(out, s.tune); break; }
        }
        for (Scored s : ok) {
            if (s.tune.brakeAware) { addIfNew(out, s.tune); break; }
        }
        for (Scored s : ok) {
            if (out.size() >= 4) break;
            addIfNew(out, s.tune);
        }
        out.add(gentler(ok.get(0).tune.copy(), 0.6));
        return out;
    }

    /**
     * Internal-model-control choice for an integrator behind dead time: with the delay predicted out, the position loop
     * is a first-order lag of time constant lambda = delay + one loop (the part that cannot be predicted), so the
     * position gain is 1 / lambda.
     */
    public double analyticKPos() {
        return 1.0 / (plant.delay + plant.loopDt + 0.01);
    }

    /** Predictive-mode settings straight from the measurements, with no search: the safety net among the finalists. */
    public TurretTune analytic() {
        TurretTune t = base.copy();
        t.brakeAware = true;
        t.brakeDelayScale = 1.0;
        t.plantDelay = plant.delay;
        t.plantAccel = plant.accel;
        t.plantDecel = plant.decel;
        t.kPos = analyticKPos();
        t.decelSafety = 0.8;
        t.maxVelocity = 0.97 * topSpeed;
        return t;
    }

    private static final Comparator<Scored> BY_SCORE = new Comparator<Scored>() {
        @Override
        public int compare(Scored a, Scored b) {
            return Double.compare(a.score, b.score);
        }
    };

    private static void addIfNew(List<TurretTune> list, TurretTune t) {
        for (TurretTune o : list) {
            if (o == t) return;
            if (Math.abs(o.kP - t.kP) <= 0.1 * o.kP && o.brakeAware == t.brakeAware
                    && Math.abs(o.maxAcceleration - t.maxAcceleration) <= 0.1 * o.maxAcceleration) return;
        }
        list.add(t);
    }

    /** Lower gain and acceleration: used when nothing scored cleanly or as a fallback that should always be stable. */
    public static TurretTune gentler(TurretTune t, double factor) {
        t.kP *= factor;
        t.kVelocityFeedback *= factor;
        t.maxAcceleration = Math.max(300, t.maxAcceleration * Math.max(0.5, factor));
        t.maxVelocity *= 0.85;
        return t;
    }

    // ------------------------------------------------------------------ distribution

    private double[] sample(boolean useMean) {
        double[] x = new double[DIM];
        for (int i = 0; i < DIM; i++) {
            double v = useMean ? mean[i] : mean[i] + sigma[i] * rng.nextGaussian();
            x[i] = clamp(v, LO[i], HI[i]);
        }
        return x;
    }

    private void updateDistribution() {
        List<Scored> sorted = new ArrayList<>(generation);
        Collections.sort(sorted, BY_SCORE);
        for (int d = 0; d < DIM; d++) {
            double m = 0;
            for (int e = 0; e < ELITES; e++) m += vectorOf(sorted.get(e))[d];
            m /= ELITES;
            double v = 0;
            for (int e = 0; e < ELITES; e++) {
                double diff = vectorOf(sorted.get(e))[d] - m;
                v += diff * diff;
            }
            double sd = Math.sqrt(v / ELITES);
            mean[d] = 0.3 * mean[d] + 0.7 * m;
            sigma[d] = Math.max(0.04 * (HI[d] - LO[d]), 0.75 * sigma[d] + 0.25 * sd);
        }
    }

    private final java.util.IdentityHashMap<TurretTune, double[]> vectors = new java.util.IdentityHashMap<>();

    private double[] vectorOf(Scored s) {
        return vectors.get(s.tune);
    }

    private TurretTune toTune(double[] x) {
        TurretTune t = base.copy();
        t.kP = Math.exp(x[0]);
        t.kVelocityFeedback = x[1] * base.kV;
        t.latency = x[2];
        t.maxAcceleration = Math.exp(x[3]);
        t.maxVelocity = x[4] * topSpeed;
        t.kA = x[5] * base.kA;
        t.brakeAware = rng.nextDouble() < x[6];
        t.brakeDelayScale = x[7];
        t.kS = base.kS * x[8];
        t.plantDelay = plant.delay;
        t.plantAccel = plant.accel * x[11];
        t.plantDecel = plant.decel;
        t.kPos = Math.exp(x[9]);
        t.decelSafety = x[10];
        if (t.brakeAware) t.maxVelocity = x[4] * topSpeed;
        vectors.put(t, x);
        return t;
    }

    // ------------------------------------------------------------------ scenario

    /**
     * Score on the fitted plant and on a slower and a faster one (delay, acceleration and start power off by about a
     * quarter), so settings that only work on exactly this model - not on a turret that differs a little - lose.
     */
    public Scored evaluate(TurretTune g) {
        double[][] variants = {{1.0, 1.0, 1.0}, {1.3, 0.75, 1.15}, {0.75, 1.25, 0.87}};
        double score = 0, settle = 0, overshoot = 0, tracking = 0, fast = 0, miss = 0;
        boolean failed = false;
        for (double[] v : variants) {
            TurretPlantModel.Params pp = plant.copy();
            pp.delay *= v[0];
            pp.accel *= v[1];
            pp.decel *= v[1];
            pp.startPower *= v[2];
            Scored r = evaluateOn(pp, g);
            if (r.failed) {
                failed = true;
                break;
            }
            score += r.score / variants.length;
            settle += r.settle / variants.length;
            overshoot += r.overshoot / variants.length;
            tracking += r.tracking / variants.length;
            fast += r.fastTracking / variants.length;
            miss += r.finalError / variants.length;
        }
        if (failed) return new Scored(g, FAIL_SCORE, 0, 0, 0, 0, 0, true);
        return new Scored(g, score, settle, overshoot, tracking, fast, miss, false);
    }

    /** Moves, a small correction pair, then two smooth tracking rides; the same shapes the hardware trial uses. */
    Scored evaluateOn(TurretPlantModel.Params plant, TurretTune g) {
        double[] targets = {searchAngle, -searchAngle, -searchAngle + 25, -searchAngle, -searchAngle + 6, -searchAngle};
        final TurretPlantModel model = new TurretPlantModel(plant, 777);
        model.reset(-searchAngle);
        SetTurretAngle c = new SetTurretAngle(model, model, 220, 1200) {
            @Override
            protected long nowNanos() {
                return model.nanos();
            }
        };
        g.applyTo(c);

        boolean failed = false;
        double settleSum = 0, overshootSum = 0;
        double lastError = 0;
        double from = -searchAngle;
        int reversals = 0;
        double lastSign = 0;

        // sit at the start point first so the controller initialises on a stopped turret
        c.setTarget(from, 1.0, false);
        for (int i = 0; i < 12; i++) {
            model.sample();
            c.update();
            model.advanceLoop();
        }

        for (double target : targets) {
            double distance = Math.abs(target - from);
            double timeout = 1.2 + distance / Math.max(30, g.maxVelocity) * 1.5;
            double dir = Math.signum(target - from);
            double t0 = model.time;
            double settledSince = -1;
            double overshoot = 0;
            double settleTime = timeout;
            reversals = 0;
            lastSign = 0;
            c.setTarget(target, 1.0, false);
            boolean settled = false;
            while (model.time - t0 < timeout) {
                model.sample();
                c.update();
                model.advanceLoop();
                double est = model.getTurretAngle();
                double err = target - est;
                overshoot = Math.max(overshoot, (model.pos - target) * dir);
                if (Math.abs(est) > safeRange + 15) { failed = true; break; }
                if (Math.abs(model.getVelocity()) > reversalSpeed && Math.abs(err) > 2.5) {
                    double s = Math.signum(model.getVelocity());
                    if (lastSign != 0 && s != lastSign) reversals++;
                    lastSign = s;
                }
                if (reversals >= 3) { failed = true; break; }
                if (Math.abs(err) <= SETTLE_TOLERANCE) {
                    if (settledSince < 0) settledSince = model.time;
                    if (model.time - settledSince >= SETTLE_HOLD) {
                        settleTime = settledSince - t0;
                        settled = true;
                        break;
                    }
                } else {
                    settledSince = -1;
                }
            }
            if (failed) break;
            lastError = Math.abs(target - model.pos);
            settleSum += settleTime + (settled ? 0 : 0.3 + 0.2 * lastError);
            overshootSum += overshoot;
            from = target;
        }

        double tracking = 0;
        double fastTracking = 0;
        if (!failed) {
            double[] r = track(model, c, from, TRACK_AMPLITUDE, TRACK_SECONDS);
            if (r[1] > 0) failed = true; else tracking = r[0];
        }
        if (!failed) {
            // a faster ride (30 deg at 0.7 Hz, peak 132 deg/s): what following a goal while the robot drives asks for
            double[] r = track(model, c, from, FAST_AMPLITUDE, FAST_SECONDS);
            if (r[1] > 0) failed = true; else fastTracking = r[0];
        }

        int moves = targets.length;
        double settle = settleSum / moves;
        double overshoot = overshootSum / moves;
        double score = failed ? FAIL_SCORE
                : settle + OVERSHOOT_WEIGHT * overshoot + TRACK_WEIGHT * tracking + FAST_TRACK_WEIGHT * fastTracking;
        return new Scored(g, score, settle, overshoot, tracking, fastTracking, lastError, failed);
    }

    /** One smooth ride 'amp * (1 - cos)' from base over a period; returns {rms error, 1 if it went unstable}. */
    private double[] track(TurretPlantModel model, SetTurretAngle c, double base0, double amp, double period) {
        double t0 = model.time;
        double sumSq = 0;
        int n = 0;
        int reversals = 0;
        double lastSign = 0;
        while (model.time - t0 < period + 0.4) {
            double t = model.time - t0;
            double ph = 2 * Math.PI * Math.min(t, period) / period;
            double target = base0 + amp * (1 - Math.cos(ph));
            c.setTarget(target, 1.0, false);
            model.sample();
            c.update();
            model.advanceLoop();
            if (t > 0.3) {
                double e = target - model.pos;
                sumSq += e * e;
                n++;
            }
            if (Math.abs(model.getVelocity()) > reversalSpeed) {
                double s = Math.signum(model.getVelocity());
                if (lastSign != 0 && s != lastSign) reversals++;
                lastSign = s;
            }
            // a ride has the velocity change sign twice by itself; three is shaking
            if (reversals >= 4 || Math.abs(target - model.pos) > 0.6 * amp + 5) {
                return new double[]{0, 1};
            }
        }
        // settle back at the base for the next ride
        for (int i = 0; i < 25; i++) {
            c.setTarget(base0, 1.0, false);
            model.sample();
            c.update();
            model.advanceLoop();
        }
        return new double[]{n > 0 ? Math.sqrt(sumSq / n) : 0, 0};
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
