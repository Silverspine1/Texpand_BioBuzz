package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Follower;

/**
 * Heading PD gains from measurement, plus the stop tolerances PathTuner uses.
 * (Translation no longer uses PD gains - the follower drives from the
 * measured drive model; PathTuner tunes its kEnd / velGain directly.)
 *
 * Pole placement on the near-target loop  e'' + A*Kd*e' + A*Kp*e = 0:
 *     Kp = wn^2/A,  Kd = 2*zeta*sqrt(Kp/A),  wn = 4/(zeta*ts)
 * Kd is re-derived from the final Kp after any clamp, so the pair keeps its
 * damping. Both PDs are then capped to a PM_MIN phase margin on the measured
 * turn step response (delay is what made high heading gains limit-cycle).
 * D is per SECOND.
 */
public class ModelTuner {

    public double aTurn = 0;                 // deg/s^2 per power (ramp regression)
    public double ksFwd = 0, ksStr = 0, ksTurn = 0;
    public double tolX = 2.0, tolY = 2.0, tolH = 2.0;   // cm, cm, deg

    public double tsH = 0.60, zetaH = 1.0, endH = 1.5;

    // turn step response per unit power: turnV / (tauTurn*s + 1) * e^(-delayTurn*s)
    public double turnV = 0, tauTurn = 0.1, delayTurn = 0.03;
    public double loopDt = 0.02;
    private static final double PM_MIN = 45;
    public double pmFast = Double.NaN, pmSlow = Double.NaN;

    private static final double KP_MIN = 1e-4, KP_MAX = 1.0;
    private static final double KD_MAX = 0.5;

    public double[] pd(double A, double ts, double zeta, double kpCap, double kpFloor) {
        if (A <= 1.0) A = 1.0;
        double wn = 4.0 / (zeta * ts);
        double kp = wn * wn / A;
        if (kpCap > 0 && kp > kpCap) kp = kpCap;
        if (kp < kpFloor) kp = kpFloor;
        kp = clamp(kp, KP_MIN, KP_MAX);
        double kd = clamp(2.0 * zeta * Math.sqrt(kp / A), 0, KD_MAX);
        return new double[]{kp, kd};
    }

    private double endCap(double ks, double tol) {
        if (ks <= 0 || tol <= 0) return 0;
        return ks / tol;
    }

    private double stictionFloor(double ks, double errScale) {
        if (ks <= 0 || errScale <= 0) return 0;
        return ks / errScale;
    }

    /** {fastP, fastD, slowP, slowD}, D per second. */
    public double[] headingGains() {
        double[] hf = pd(aTurn, tsH, zetaH, 0, stictionFloor(ksTurn, 10));
        double[] hs = pd(aTurn, tsH * endH, zetaH, endCap(ksTurn, tolH), stictionFloor(ksTurn, 3 * tolH));
        hf = marginCap(hf, aTurn, zetaH);
        hs = marginCap(capTo(hs, hf, aTurn, zetaH), aTurn, zetaH);
        pmFast = phaseMargin(hf[0], hf[1]);
        pmSlow = phaseMargin(hs[0], hs[1]);
        return new double[]{hf[0], hf[1], hs[0], hs[1]};
    }

    /** The soft PD must never be stiffer than the fast one. */
    private double[] capTo(double[] soft, double[] fast, double A, double zeta) {
        if (soft[0] <= fast[0]) return soft;
        if (A <= 1.0) A = 1.0;
        double kp = fast[0];
        return new double[]{kp, clamp(2.0 * zeta * Math.sqrt(kp / A), 0, KD_MAX)};
    }

    /** Lowers kp (kd re-derived) until the loop has PM_MIN phase margin.
     *  Overrides the stiction floor - the kS feedforward handles friction. */
    private double[] marginCap(double[] g, double A, double zeta) {
        if (turnV <= 0) return g;
        if (A <= 1.0) A = 1.0;
        double kp = g[0], kd = g[1];
        for (int n = 0; n < 80 && phaseMargin(kp, kd) < PM_MIN && kp > KP_MIN; n++) {
            kp *= 0.9;
            kd = clamp(2.0 * zeta * Math.sqrt(kp / A), 0, KD_MAX);
        }
        return new double[]{kp, kd};
    }

    /** Phase margin (deg) of PD on turnV*e^(-sT)/(s(tau s+1)), T = delay + one loop. */
    public double phaseMargin(double kp, double kd) {
        if (turnV <= 0) return Double.NaN;
        double tau = Math.max(0.005, tauTurn), T = Math.max(0, delayTurn) + loopDt;
        double lo = Math.log(0.05), hi = Math.log(2000);
        for (int n = 0; n < 60; n++) {
            double mid = 0.5 * (lo + hi);
            if (loopMag(Math.exp(mid), kp, kd, tau) > 1) lo = mid; else hi = mid;
        }
        double w = Math.exp(0.5 * (lo + hi));
        double ph = Math.atan2(w * kd, kp) - Math.PI / 2 - Math.atan(w * tau) - w * T;
        return 180 + Math.toDegrees(ph);
    }

    private double loopMag(double w, double kp, double kd, double tau) {
        return turnV * Math.hypot(kp, w * kd) / (w * Math.hypot(1, w * tau));
    }

    private static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
}
