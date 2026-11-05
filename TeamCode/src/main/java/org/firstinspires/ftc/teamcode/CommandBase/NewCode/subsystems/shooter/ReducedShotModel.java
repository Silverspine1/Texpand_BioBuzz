package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.List;
import java.util.Map;

/**
 * Data of the REDUCED shot targeter: the robot JSON written by the Python side
 * ({@code python -m ftc_optimizer.shotcontrol.algos.reduced} ->
 * {@code outputs/shotcontrol/algos/robot_model_red_audience.json}, schema
 * {@code ftc_optimizer.shotcontrol.robot_model/1}; layout in docs/SHOT_ALGO_DESIGN.md section 7).
 *
 * <p>On the Control Hub the file lives in {@code /sdcard/FIRST/settings/} (the folder
 * {@code AppUtil.getInstance().getSettingsFile(name)} resolves to, the same place as
 * {@code shooter_tune.properties} and {@code turret_tune.properties}). Copy it with
 * {@code adb push robot_model_red_audience.json /sdcard/FIRST/settings/}. Recommended (R4):
 * {@code adb push outputs/shotcontrol/algos/robot_model_red_audience_r10.json /sdcard/FIRST/settings/} and
 * {@code ShotControlTuning.modelFile = "robot_model_red_audience_r10.json"} (same layout; optional keys
 * {@code occ.bearing_calib} and {@code aim.meta.stay} are informational). An optional field-tune export
 * ({@code under_shooter_field_tune/1}, docs/UNDER_SHOOTER_TUNING.md) placed next to it as
 * {@link #FIELD_TUNE_FILE} replaces the {@code shot} block ({@link #applyFieldTune}).
 *
 * <p>All arrays are flat, row-major: aim {@code [i_d][i_vr][i_vt]}, shot {@code [i_rpm][i_hood]},
 * wheel {@code [i_rpm][i_spacing]}. Everything is allocated here, once; the solver allocates nothing.
 * Pure Java (no FTC SDK), so it compiles and runs in a plain JVM for the parity test.
 */
public final class ReducedShotModel {

    public static final String SCHEMA = "ftc_optimizer.shotcontrol.robot_model/1";
    public static final String FIELD_TUNE_SCHEMA = "under_shooter_field_tune/1";
    /** Default file names in /sdcard/FIRST/settings/ (the model name is configurable: ShotControlTuning.modelFile). */
    public static final String MODEL_FILE = "robot_model_red_audience.json";
    /** Recommended model (review R4: aim table with P(stay in cell), bearing calibration folded into occ.f). The robot loads
     *  {@code ShooterController.BUILT_IN_MODEL} from the app assets (currently r11: rising entries accepted, 2026-11-04). */
    public static final String MODEL_FILE_R4 = "robot_model_red_audience_r11.json";
    public static final String FIELD_TUNE_FILE = "under_shooter_field_tune.json";

    // ---- constants --------------------------------------------------------------------------------------------
    public double poseS, fireS, pivotFwdIn, pivotRightIn, wheelFwdIn, wheelHeightIn, armExtraIn;
    public double aimX, aimY, aimZ, rpmMax, droopMinRpm, turretLeadS;
    public int nInv, nDroop, nMuzzle;
    public String goal = "";

    // ---- aim table (velocity-compensated, exit space) -----------------------------------------------------------
    public double[] aimD, aimVr, aimVt;
    public int nD, nVr, nVt;
    public double okMin;
    public boolean[] aimOk;
    /** Field index into {@link #aim}: speed, angle, yaw, tof, p, slope, rpm, hood. */
    public static final int A_SPEED = 0, A_ANGLE = 1, A_YAW = 2, A_TOF = 3, A_P = 4, A_SLOPE = 5, A_RPM = 6, A_HOOD = 7;
    public static final String[] AIM_FIELDS = {"speed", "angle", "yaw", "tof", "p", "slope", "rpm", "hood"};
    public final double[][] aim = new double[8][];

    // ---- shot model: bicubic Hermite node data --------------------------------------------------------------
    public double[] shotRpm, shotHood;
    public int nR, nH;
    public static final int S_SPEED = 0, S_ANGLE = 1, S_DROP = 2;
    /** shot[field][0..3] = F, Fx (d/drpm), Fy (d/dhood), Fxy, each flat [i_rpm * nH + j_hood]. */
    public final double[][][] shot = new double[3][4][];
    /** Where the shot block came from (robot JSON or the field-tune export), for telemetry. */
    public String shotSource = "robot model";

    // ---- wheel (burst droop) table ------------------------------------------------------------------------------
    public double[] wheelRpm, wheelSpacing;
    public int nWR, nWS;
    public double[] deltaK1, deltaKn, residK1, residKn;

    // ---- optional hive-occlusion / stay map on the turret-pivot position (reduced.occ_stay_map) -----------------
    public boolean hasOcc;
    public double[] occX, occY, occF, occStay;
    /** Informational (R4, optional): occ.bearing_calib = [PHI0 deg, A] already folded into occF (null if absent),
     *  and aim.meta.stay = the aim table's p includes P(stay in cell). Neither changes the solve. */
    public double[] occBearingCalib;
    public boolean aimHasStay;

    private ReducedShotModel() {
    }

    // =========================================================================================================
    // loading
    // =========================================================================================================

    public static String readText(File f) throws IOException {
        InputStream in = new FileInputStream(f);
        try {
            return readText(in);
        } finally {
            in.close();
        }
    }

    public static String readText(InputStream in) throws IOException {
        ByteArrayOutputStream bo = new ByteArrayOutputStream(1 << 16);
        byte[] buf = new byte[1 << 15];
        int n;
        while ((n = in.read(buf)) > 0) bo.write(buf, 0, n);
        return new String(bo.toByteArray(), "UTF-8");
    }

    public static ReducedShotModel load(File modelFile) throws IOException {
        return fromJson(readText(modelFile));
    }

    /** Parses the robot JSON (schema {@link #SCHEMA}). Throws with a readable message when it does not match. */
    public static ReducedShotModel fromJson(String text) {
        Object root = ShotJson.parse(text);
        String schema = ShotJson.str(root, "schema", "");
        if (!SCHEMA.equals(schema)) throw new IllegalArgumentException("robot model schema '" + schema + "', expected " + SCHEMA);
        ReducedShotModel m = new ReducedShotModel();

        Map<String, Object> c = ShotJson.obj(root, "constants");
        m.poseS = ShotJson.num(c, "pose_s");
        m.fireS = ShotJson.num(c, "fire_s");
        m.pivotFwdIn = ShotJson.num(c, "pivot_fwd_in");
        m.pivotRightIn = ShotJson.num(c, "pivot_right_in");
        m.wheelFwdIn = ShotJson.num(c, "wheel_fwd_in");
        m.wheelHeightIn = ShotJson.num(c, "wheel_height_in", 10.0);
        m.armExtraIn = ShotJson.num(c, "arm_extra_in");
        m.aimX = ShotJson.num(c, "aim_x");
        m.aimY = ShotJson.num(c, "aim_y");
        m.aimZ = ShotJson.num(c, "aim_z", Double.NaN);
        m.rpmMax = ShotJson.num(c, "rpm_max");
        m.nInv = (int) ShotJson.num(c, "n_inv");
        m.nDroop = (int) ShotJson.num(c, "n_droop");
        m.nMuzzle = (int) ShotJson.num(c, "n_muzzle", 2);       // Python: C.get("n_muzzle", 2)
        m.droopMinRpm = ShotJson.num(c, "droop_min_rpm");
        m.turretLeadS = ShotJson.num(c, "turret_lead_s");
        m.goal = ShotJson.str(c, "goal", "");

        Map<String, Object> a = ShotJson.obj(root, "aim");
        m.aimD = ShotJson.vec(ShotJson.list(a, "d"));
        m.aimVr = ShotJson.vec(ShotJson.list(a, "vr"));
        m.aimVt = ShotJson.vec(ShotJson.list(a, "vt"));
        m.nD = m.aimD.length;
        m.nVr = m.aimVr.length;
        m.nVt = m.aimVt.length;
        if (m.nD < 2 || m.nVr < 2 || m.nVt < 2) throw new IllegalArgumentException("aim grid needs >= 2 nodes per axis");
        m.okMin = ShotJson.num(a, "ok_min", 0.5);              // Python: at.get("ok_min", 0.5)
        if (ShotJson.has(a, "meta")) {
            Object st = ShotJson.obj(a, "meta").get("stay");
            m.aimHasStay = Boolean.TRUE.equals(st);
        }
        double[] ok = ShotJson.flat3(ShotJson.list(a, "ok"), m.nD, m.nVr, m.nVt, "aim.ok");
        m.aimOk = new boolean[ok.length];
        for (int k = 0; k < ok.length; k++) m.aimOk[k] = ok[k] == 1.0;      // Python: OKA[...] != 1 -> skip
        for (int f = 0; f < 8; f++) {
            m.aim[f] = ShotJson.flat3(ShotJson.list(a, AIM_FIELDS[f]), m.nD, m.nVr, m.nVt, "aim." + AIM_FIELDS[f]);
        }

        Map<String, Object> sh = ShotJson.obj(root, "shot");
        m.shotRpm = ShotJson.vec(ShotJson.list(sh, "rpm"));
        m.shotHood = ShotJson.vec(ShotJson.list(sh, "hood"));
        m.nR = m.shotRpm.length;
        m.nH = m.shotHood.length;
        String[] sf = {"speed", "angle", "drop"};
        for (int f = 0; f < 3; f++) {
            List<Object> four = ShotJson.list(sh, sf[f]);
            if (four.size() != 4) throw new IllegalArgumentException("shot." + sf[f] + " must be [F, Fx, Fy, Fxy]");
            for (int q = 0; q < 4; q++) {
                @SuppressWarnings("unchecked")
                List<Object> grid = (List<Object>) four.get(q);
                m.shot[f][q] = ShotJson.flat2(grid, m.nR, m.nH, "shot." + sf[f] + "[" + q + "]");
            }
        }

        Map<String, Object> w = ShotJson.obj(root, "wheel");
        m.wheelRpm = ShotJson.vec(ShotJson.list(w, "rpm"));
        m.wheelSpacing = ShotJson.vec(ShotJson.list(w, "spacing_s"));
        m.nWR = m.wheelRpm.length;
        m.nWS = m.wheelSpacing.length;
        m.deltaK1 = ShotJson.flat2(ShotJson.list(w, "delta_k1"), m.nWR, m.nWS, "wheel.delta_k1");
        m.deltaKn = ShotJson.flat2(ShotJson.list(w, "delta_kn"), m.nWR, m.nWS, "wheel.delta_kn");
        m.residK1 = ShotJson.flat2(ShotJson.list(w, "resid_k1"), m.nWR, m.nWS, "wheel.resid_k1");
        m.residKn = ShotJson.flat2(ShotJson.list(w, "resid_kn"), m.nWR, m.nWS, "wheel.resid_kn");

        if (ShotJson.has(root, "occ")) {             // Python: M.get("occ") is not None
            Map<String, Object> o = ShotJson.obj(root, "occ");
            m.occX = ShotJson.vec(ShotJson.list(o, "x"));
            m.occY = ShotJson.vec(ShotJson.list(o, "y"));
            m.occF = ShotJson.flat2(ShotJson.list(o, "f"), m.occX.length, m.occY.length, "occ.f");
            m.occStay = ShotJson.flat2(ShotJson.list(o, "stay"), m.occX.length, m.occY.length, "occ.stay");
            m.hasOcc = true;
            if (o.get("bearing_calib") instanceof List) m.occBearingCalib = ShotJson.vec(ShotJson.list(o, "bearing_calib"));
        }
        return m;
    }

    // =========================================================================================================
    // field-tune export -> shot block
    // =========================================================================================================

    /**
     * Replaces the {@code shot} block with the release table of a field-tune export
     * ({@code under_shooter_field_tune/1}: {@code table.rpm_grid}, {@code table.hood_grid},
     * {@code table.fields.exit_speed_ips / exit_angle_deg / wheel_rpm_drop} indexed {@code [i_rpm][j_hood]}).
     * The node slopes are computed exactly as the Python {@code ShotTable._prep} does (invalid cells filled from
     * the nearest valid cell, then {@code np.gradient} with second-order edges), so the result equals the Python
     * export of the same table. The aim table is in EXIT space and stays valid (SHOT_ALGO_DESIGN section 8;
     * rebuild it offline after a large stiffness change). Allocates; call at init only.
     */
    public void applyFieldTune(String text) {
        Object root = ShotJson.parse(text);
        String schema = ShotJson.str(root, "schema", "");
        if (!FIELD_TUNE_SCHEMA.equals(schema)) {
            throw new IllegalArgumentException("field-tune schema '" + schema + "', expected " + FIELD_TUNE_SCHEMA);
        }
        Map<String, Object> t = ShotJson.obj(root, "table");
        double[] rg = ShotJson.vec(ShotJson.list(t, "rpm_grid"));
        double[] hg = ShotJson.vec(ShotJson.list(t, "hood_grid"));
        int nr = rg.length, nh = hg.length;
        if (nr < 2 || nh < 2) throw new IllegalArgumentException("field-tune table needs >= 2 nodes per axis");
        Map<String, Object> f = ShotJson.obj(t, "fields");
        double[] spd = ShotJson.flat2(ShotJson.list(f, "exit_speed_ips"), nr, nh, "exit_speed_ips");
        double[] ang = ShotJson.flat2(ShotJson.list(f, "exit_angle_deg"), nr, nh, "exit_angle_deg");
        double[] drp = ShotJson.has(f, "wheel_rpm_drop")
                ? ShotJson.flat2(ShotJson.list(f, "wheel_rpm_drop"), nr, nh, "wheel_rpm_drop")
                : new double[nr * nh];
        boolean[] ok = new boolean[nr * nh];
        int nOk = 0;
        for (int k = 0; k < ok.length; k++) {
            ok[k] = !Double.isNaN(spd[k]) && !Double.isInfinite(spd[k]) && !Double.isNaN(ang[k]) && !Double.isInfinite(ang[k]);
            if (ok[k]) nOk++;
        }
        if (nOk == 0) throw new IllegalArgumentException("field-tune table has no valid cell");
        double[][] src = {spd, ang, drp};
        double[][][] out = new double[3][4][];
        for (int q = 0; q < 3; q++) {
            double[] F = fillNanNearest(src[q], ok, nr, nh);
            double[] fx = gradient(F, rg, nr, nh, 0);
            double[] fy = gradient(F, hg, nr, nh, 1);
            double[] fxy = gradient(fx, hg, nr, nh, 1);
            out[q][0] = F;
            out[q][1] = fx;
            out[q][2] = fy;
            out[q][3] = fxy;
        }
        // commit only after everything parsed
        shotRpm = rg;
        shotHood = hg;
        nR = nr;
        nH = nh;
        for (int q = 0; q < 3; q++) for (int k = 0; k < 4; k++) shot[q][k] = out[q][k];
        Object p = ((Map<?, ?>) root).get("parameters");
        shotSource = "field tune" + (p instanceof Map ? " " + paramText((Map<?, ?>) p) : "");
    }

    private static String paramText(Map<?, ?> p) {
        StringBuilder b = new StringBuilder("(");
        String[] keys = {"mu_wheel", "mu_guide", "feed_speed_m_s", "speed_scale"};
        for (String k : keys) {
            Object v = p.get(k);
            if (v instanceof Double) {
                if (b.length() > 1) b.append(' ');
                b.append(k).append('=').append(String.format(java.util.Locale.US, "%.3f", (Double) v));
            }
        }
        return b.append(')').toString();
    }

    /** Python {@code tables._fill_nan_nearest}: invalid cells take the nearest valid cell (index metric, ties to
     *  the first valid cell in row-major order). */
    static double[] fillNanNearest(double[] F, boolean[] ok, int n0, int n1) {
        double[] out = F.clone();
        for (int i = 0; i < n0; i++) {
            for (int j = 0; j < n1; j++) {
                if (ok[i * n1 + j]) continue;
                long best = Long.MAX_VALUE;
                int bk = -1;
                for (int vi = 0; vi < n0; vi++) {
                    for (int vj = 0; vj < n1; vj++) {
                        if (!ok[vi * n1 + vj]) continue;
                        long d2 = (long) (i - vi) * (i - vi) + (long) (j - vj) * (j - vj);
                        if (d2 < best) { best = d2; bk = vi * n1 + vj; }
                    }
                }
                out[i * n1 + j] = F[bk];
            }
        }
        return out;
    }

    /**
     * numpy {@code np.gradient(F, coords, axis=axis, edge_order=2 if n > 2 else 1)} for a [n0][n1] row-major array,
     * including numpy's reduction to the uniform-spacing formulas when every grid step is exactly equal.
     */
    static double[] gradient(double[] F, double[] x, int n0, int n1, int axis) {
        int n = axis == 0 ? n0 : n1;
        int stride = axis == 0 ? n1 : 1;
        int lines = axis == 0 ? n1 : n0;
        int lineStep = axis == 0 ? 1 : n1;
        double[] dx = new double[n - 1];
        boolean uniform = true;
        for (int k = 0; k < n - 1; k++) {
            dx[k] = x[k + 1] - x[k];
            if (dx[k] != dx[0]) uniform = false;
        }
        double h = dx[0];
        double[] g = new double[F.length];
        for (int L = 0; L < lines; L++) {
            int base = L * lineStep;
            // interior
            for (int k = 1; k < n - 1; k++) {
                double fm = F[base + (k - 1) * stride], f0 = F[base + k * stride], fp = F[base + (k + 1) * stride];
                if (uniform) {
                    g[base + k * stride] = (fp - fm) / (2.0 * h);
                } else {
                    double dx1 = dx[k - 1], dx2 = dx[k];
                    double a = -(dx2) / (dx1 * (dx1 + dx2));
                    double b = (dx2 - dx1) / (dx1 * dx2);
                    double c = dx1 / (dx2 * (dx1 + dx2));
                    g[base + k * stride] = a * fm + b * f0 + c * fp;
                }
            }
            if (n > 2) {          // edge_order = 2
                double f0 = F[base], f1 = F[base + stride], f2 = F[base + 2 * stride];
                double a, b, c;
                if (uniform) {
                    a = -1.5 / h; b = 2.0 / h; c = -0.5 / h;
                } else {
                    double dx1 = dx[0], dx2 = dx[1];
                    a = -(2.0 * dx1 + dx2) / (dx1 * (dx1 + dx2));
                    b = (dx1 + dx2) / (dx1 * dx2);
                    c = -dx1 / (dx2 * (dx1 + dx2));
                }
                g[base] = a * f0 + b * f1 + c * f2;
                double e3 = F[base + (n - 3) * stride], e2 = F[base + (n - 2) * stride], e1 = F[base + (n - 1) * stride];
                if (uniform) {
                    a = 0.5 / h; b = -2.0 / h; c = 1.5 / h;
                } else {
                    double dx1 = dx[n - 3], dx2 = dx[n - 2];
                    a = (dx2) / (dx1 * (dx1 + dx2));
                    b = -(dx2 + dx1) / (dx1 * dx2);
                    c = (2.0 * dx2 + dx1) / (dx2 * (dx1 + dx2));
                }
                g[base + (n - 1) * stride] = a * e3 + b * e2 + c * e1;
            } else {              // edge_order = 1, n == 2
                double d0 = (F[base + stride] - F[base]) / dx[0];
                g[base] = d0;
                g[base + stride] = d0;
            }
        }
        return g;
    }

    /** One-line description for telemetry. */
    public String describe() {
        String occ = !hasOcc ? "" : occBearingCalib != null && occBearingCalib.length >= 2
                ? String.format(java.util.Locale.US, " +occ(bearing %.0f/%.4f)", occBearingCalib[0], occBearingCalib[1]) : " +occ";
        return String.format(java.util.Locale.US, "goal %s aim (%.1f, %.1f) in, aim %dx%dx%d%s, shot %dx%d [%s]%s rpm_max %.0f",
                goal, aimX, aimY, nD, nVr, nVt, aimHasStay ? " +stay" : "", nR, nH, shotSource, occ, rpmMax);
    }
}
