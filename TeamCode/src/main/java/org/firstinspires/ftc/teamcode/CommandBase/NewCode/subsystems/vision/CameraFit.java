package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

/**
 * Fits the camera model to balls placed at measured spots. Pure Java (Nelder-Mead, no libraries).
 *
 * A mark is: the pixel centre (u, v) of ONE ball seen by the camera, and where that ball's centre really is on the
 * floor, in cm from the chassis centre (forward, left). For a candidate camera the pixel is back-projected to the
 * floor and compared with the measured spot; the fit adjusts pitch, horizontal FOV and the lens mix so the error
 * (cm) over all marks is smallest. The mount position (forward / left / height) is taken as MEASURED and left alone.
 *
 * Use marks that spread over the picture: some straight ahead at different distances AND some off to each side -
 * the FOV and lens shape are only visible off-axis. 5-8 marks is plenty.
 */
public final class CameraFit {

    public static final class Mark {
        public final double u, v, fwdCm, leftCm;
        public Mark(double u, double v, double fwdCm, double leftCm) { this.u = u; this.v = v; this.fwdCm = fwdCm; this.leftCm = leftCm; }
    }

    public static final class Result {
        public double pitchDeg, hfovDeg, mix, rmsCm, rmsBeforeCm;
        public boolean ok;
        public String note = "";
    }

    private CameraFit() { }

    /** RMS floor error (cm) of the marks for a candidate camera. */
    public static double rms(VisionParams base, List<Mark> marks, double pitch, double hfov, double mix) {
        VisionParams c = copyOf(base);
        c.camPitchDownDeg = pitch;
        c.hfovDeg = hfov;
        c.lensRectMix = mix;
        BallCamera cam = new BallCamera(c, false);
        double sum = 0;
        int n = 0;
        for (Mark m : marks) {
            double[] g = cam.groundPoint(m.u, m.v, c.ballRadiusCm);
            double e = g == null ? 200.0 : Math.hypot(g[0] - m.fwdCm, g[1] - m.leftCm);
            sum += e * e;
            n++;
        }
        return n == 0 ? 0 : Math.sqrt(sum / n);
    }

    public static Result fit(VisionParams base, List<Mark> marks, boolean fitMix) {
        Result r = new Result();
        r.pitchDeg = base.camPitchDownDeg; r.hfovDeg = base.hfovDeg; r.mix = base.lensRectMix;
        if (marks.size() < 3) { r.note = "need at least 3 marks (5-8 is better)"; return r; }
        r.rmsBeforeCm = rms(base, marks, r.pitchDeg, r.hfovDeg, r.mix);
        double[] x0 = {base.camPitchDownDeg, base.hfovDeg, base.lensRectMix};
        double[] step = {4.0, 10.0, 0.25};
        int dim = fitMix ? 3 : 2;
        double best = Double.MAX_VALUE;
        double[] bestX = x0.clone();
        // a few restarts: the surface has a shallow valley between FOV and mix
        double[][] starts = {x0, {x0[0], 100, 0.0}, {x0[0], 120, 0.0}, {x0[0], 110, 0.5}, {x0[0] + 5, 120, 0.2}, {x0[0] - 5, 100, 0.8}};
        for (double[] s : starts) {
            double[] x = nelderMead(base, marks, s.clone(), step, dim, fitMix);
            double f = cost(base, marks, x, fitMix);
            if (f < best) { best = f; bestX = x; }
        }
        r.pitchDeg = bestX[0]; r.hfovDeg = bestX[1]; r.mix = fitMix ? clamp(bestX[2], 0, 1) : base.lensRectMix;
        r.rmsCm = rms(base, marks, r.pitchDeg, r.hfovDeg, r.mix);
        r.ok = r.rmsCm < r.rmsBeforeCm;
        r.note = r.ok ? "fit improved the error" : "fit did not improve - check the marks";
        return r;
    }

    private static double cost(VisionParams base, List<Mark> marks, double[] x, boolean fitMix) {
        double pitch = clamp(x[0], base.camPitchDownDeg - 20, base.camPitchDownDeg + 20);
        double hfov = clamp(x[1], 60, 170);
        double mix = fitMix ? clamp(x[2], 0, 1) : base.lensRectMix;
        double pen = (x[0] - pitch) * (x[0] - pitch) + (x[1] - hfov) * (x[1] - hfov) + (fitMix ? (x[2] - mix) * (x[2] - mix) * 100 : 0);
        return rms(base, marks, pitch, hfov, mix) + 0.05 * (fitMix ? mix : 0) + 10 * pen;   // weak pull toward the equidistant lens
    }

    private static double[] nelderMead(VisionParams base, List<Mark> marks, double[] start, double[] step, int dim, boolean fitMix) {
        int n = dim;
        double[][] sp = new double[n + 1][];
        double[] fv = new double[n + 1];
        for (int i = 0; i <= n; i++) {
            sp[i] = start.clone();
            if (i > 0) sp[i][i - 1] += step[i - 1];
            fv[i] = cost(base, marks, sp[i], fitMix);
        }
        for (int it = 0; it < 220; it++) {
            int lo = 0, hi = 0;
            for (int i = 1; i <= n; i++) { if (fv[i] < fv[lo]) lo = i; if (fv[i] > fv[hi]) hi = i; }
            if (Math.abs(fv[hi] - fv[lo]) < 1e-6) break;
            double[] cen = new double[start.length];
            for (int i = 0; i <= n; i++) if (i != hi) for (int d = 0; d < n; d++) cen[d] += sp[i][d] / n;
            for (int d = n; d < start.length; d++) cen[d] = start[d];
            double[] refl = along(cen, sp[hi], -1.0, start.length);
            double fr = cost(base, marks, refl, fitMix);
            if (fr < fv[lo]) {
                double[] exp = along(cen, sp[hi], -2.0, start.length);
                double fe = cost(base, marks, exp, fitMix);
                if (fe < fr) { sp[hi] = exp; fv[hi] = fe; } else { sp[hi] = refl; fv[hi] = fr; }
            } else if (fr < fv[hi]) {
                sp[hi] = refl; fv[hi] = fr;
            } else {
                double[] con = along(cen, sp[hi], 0.5, start.length);
                double fc = cost(base, marks, con, fitMix);
                if (fc < fv[hi]) { sp[hi] = con; fv[hi] = fc; }
                else for (int i = 0; i <= n; i++) if (i != lo) {
                    for (int d = 0; d < n; d++) sp[i][d] = sp[lo][d] + 0.5 * (sp[i][d] - sp[lo][d]);
                    fv[i] = cost(base, marks, sp[i], fitMix);
                }
            }
        }
        int lo = 0;
        for (int i = 1; i <= n; i++) if (fv[i] < fv[lo]) lo = i;
        return sp[lo];
    }

    private static double[] along(double[] c, double[] w, double t, int len) {
        double[] out = new double[len];
        for (int d = 0; d < len; d++) out[d] = c[d] + t * (w[d] - c[d]);
        return out;
    }

    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }

    static VisionParams copyOf(VisionParams b) {
        VisionParams c = new VisionParams();
        c.workWidth = b.workWidth; c.workHeight = b.workHeight; c.hfovDeg = b.hfovDeg; c.lensRectMix = b.lensRectMix;
        c.camForwardCm = b.camForwardCm; c.camLeftCm = b.camLeftCm; c.camHeightCm = b.camHeightCm;
        c.camPitchDownDeg = b.camPitchDownDeg; c.camYawDeg = b.camYawDeg; c.ballRadiusCm = b.ballRadiusCm;
        c.lateralSign = b.lateralSign;
        return c;
    }

    public static List<Mark> newList() { return new ArrayList<>(); }
}
