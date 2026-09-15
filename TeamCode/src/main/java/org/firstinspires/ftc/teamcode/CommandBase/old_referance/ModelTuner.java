package org.firstinspires.ftc.teamcode.CommandBase.old_referance;

/**
 * Model-based PD gain computation. Everything here is driven by on-robot
 * measurements made by PathTuner - there are no hand-set numbers.
 *
 * Inputs (all measured):
 *   aFwd / aStr / aTurn - acceleration produced per unit motor power
 *                         (cm/s^2 per power, deg/s^2 per power for turn),
 *                         from a least-squares fit of the full-power ramp.
 *   ksFwd / ksStr / ksTurn - breakaway (static friction) power per axis,
 *                            from a slow power ramp until movement.
 *   tolX / tolY / tolH  - stop tolerances, derived from the stiction position
 *                         quantum (smallest reliable move), not hand-picked.
 *   tsX / tsY / tsH     - settling-time targets, driven down by PathTuner's
 *                         probe search until each axis just meets tolerance.
 *
 * Gain math - standard pole placement on the near-target 2nd-order loop
 *     error'' + A*Kd*error' + A*Kp*error = 0
 *     Kp = wn^2/A,  Kd = 2*zeta*wn/A,  wn = 4/(zeta*ts),  zeta = 1
 * with two fixes over the old version:
 *
 * 1. ANTI-TWITCH ENDPOINT CAP. Endpoint twitch is stick-slip: PD power below
 *    static friction leaves the robot stuck, error integrates nowhere, then a
 *    disturbance breaks it loose and it lurches. The cure is to guarantee that
 *    inside the tolerance box the commanded power stays BELOW breakaway, so
 *    the robot simply stays put once it is in tolerance:
 *        Kp_end <= kS / tol
 *    Endpoint Kp is min(pole-placement Kp, that cap).
 *
 * 2. DAMPING PRESERVED THROUGH CLAMPS. If Kp is capped/clamped, Kd is
 *    re-derived from the ACTUAL Kp so the pair stays critically damped:
 *        Kd = 2*zeta*sqrt(Kp/A)
 *    (The old code clamped Kp and Kd independently, which silently produced
 *    underdamped pairs - the source of the wobble.)
 *
 * Refs: Astrom & Murray, "Feedback Systems", ch.10 (PID / pole placement).
 */
public class ModelTuner {

    // Measured accel-per-unit-power (regression over full-power ramp).
    public double aFwd  = 0;   // robot Y, cm/s^2 per power
    public double aStr  = 0;   // robot X, cm/s^2 per power
    public double aTurn = 0;   // heading, deg/s^2 per power

    // Measured breakaway (static friction) power per axis, 0..1.
    public double ksFwd  = 0;
    public double ksStr  = 0;
    public double ksTurn = 0;

    // Derived stop tolerances (PathTuner computes these from the stiction
    // position quantum; they are not user knobs).
    public double tolX = 2.0;   // cm
    public double tolY = 2.0;   // cm
    public double tolH = 2.0;   // deg

    // Settling-time targets, driven by PathTuner's search.
    public double zeta = 1.0;
    public double tsX  = 0.70;
    public double tsY  = 0.70;
    public double tsH  = 0.60;

    // On-path tracking is stiffer than the endpoint hold by this ratio
    // (tracking has feed-forward help; the endpoint loop is on its own).
    private static final double PATH_RATIO = 0.75;

    // Hard sanity bounds only - real shaping comes from measurement.
    private static final double KP_MIN = 1e-4, KP_MAX = 1.0;
    private static final double KD_MAX = 0.5;

    /**
     * {Kp, Kd} for one axis. If kpCap > 0 the proportional gain is limited to
     * it (anti-twitch endpoint cap). Kd is always re-derived from the final
     * Kp so the pair stays at the requested zeta no matter what clamped.
     */
    public double[] pd(double A, double ts, double kpCap, double kpFloor) {
        if (A <= 1.0) A = 1.0;                    // degenerate measurement guard
        double wn = 4.0 / (zeta * ts);
        double kp = wn * wn / A;
        if (kpCap > 0 && kp > kpCap) kp = kpCap;
        if (kp < kpFloor) kp = kpFloor;
        kp = clamp(kp, KP_MIN, KP_MAX);
        // critical damping for the ACTUAL kp: wn' = sqrt(kp*A), kd = 2*zeta*wn'/A
        double kd = clamp(2.0 * zeta * Math.sqrt(kp / A), 0, KD_MAX);
        return new double[]{kp, kd};
    }

    /** Anti-twitch cap: power at the tolerance edge stays under breakaway. */
    private double endCap(double ks, double tol) {
        if (ks <= 0 || tol <= 0) return 0;        // no measurement -> no cap
        return ks / tol;
    }

    /**
     * Full 12-gain vector in PathTuner's order:
     *  0,1 onPathX   2,3 onPathY   4,5 endX   6,7 endY   8,9 hdgFast   10,11 hdgSlow
     */
    /** Floor so the loop can actually break static friction: at an error of
     *  errScale the commanded power must reach breakaway, or the robot creeps
     *  forever and every probe times out. */
    private double stictionFloor(double ks, double errScale) {
        if (ks <= 0 || errScale <= 0) return 0;
        return ks / errScale;
    }

    public double[] gains() {
        // on-path: power must reach breakaway within ~10cm/10deg of error
        double[] px = pd(aStr,  tsX * PATH_RATIO, 0, stictionFloor(ksStr, 10));
        double[] py = pd(aFwd,  tsY * PATH_RATIO, 0, stictionFloor(ksFwd, 10));
        // endpoint: floor at 3x tolerance (still converges), cap at 1x
        // tolerance (no stick-slip twitch inside the box)
        double[] ex = pd(aStr,  tsX, endCap(ksStr, tolX), stictionFloor(ksStr, 3 * tolX));
        double[] ey = pd(aFwd,  tsY, endCap(ksFwd, tolY), stictionFloor(ksFwd, 3 * tolY));
        // The endpoint loop must NEVER be stiffer than the on-path loop - the
        // follower switches to it near the path end, and a stiffer endpoint
        // gain makes the robot visibly speed up right before the target and
        // overshoot. Cap it, and re-derive Kd so damping stays critical.
        ex = capToPath(ex, px, aStr);
        ey = capToPath(ey, py, aFwd);
        double[] hf = pd(aTurn, tsH, 0, stictionFloor(ksTurn, 10));
        // Slow heading profile: own pole placement at a longer settling time
        // (not a scaled copy of fast - a scaled Kp with unscaled Kd is no
        // longer critically damped and steps torque at the profile switch).
        double[] hs = pd(aTurn, tsH * 1.5, endCap(ksTurn, tolH), stictionFloor(ksTurn, 3 * tolH));

        return new double[]{
                px[0], px[1], py[0], py[1],
                ex[0], ex[1], ey[0], ey[1],
                hf[0], hf[1], hs[0], hs[1]
        };
    }

    private double[] capToPath(double[] end, double[] path, double A) {
        if (end[0] <= path[0]) return end;
        if (A <= 1.0) A = 1.0;
        double kp = path[0];
        double kd = clamp(2.0 * zeta * Math.sqrt(kp / A), 0, KD_MAX);
        return new double[]{kp, kd};
    }

    private static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
}
