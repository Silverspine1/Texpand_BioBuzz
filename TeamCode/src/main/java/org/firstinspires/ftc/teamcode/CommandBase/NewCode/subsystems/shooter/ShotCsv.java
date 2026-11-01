package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

import java.util.Locale;

/**
 * One CSV line per shot in the FIELD-TUNE format ({@code ftc_optimizer.under_shooter.field_tune.io_csv}, spec in
 * docs/UNDER_SHOOTER_TUNING.md section 6/7; the first 13 columns are exactly {@code practice/shot_log.py}
 * FIELD_TUNE_COLUMNS, then the analysis columns the practice log also uses plus the gate's). Pure Java, no SDK.
 * Empty cell = not observed. Free columns after the 13 are ignored by {@code field_tune fit}.
 *
 * <p>Outcome labels the operator enters: HIT, SHORT, LONG (field-tune outcomes {@code hit|short|long}), and LEFT /
 * RIGHT which the format has no word for: written as outcome {@code miss} with {@code note} "LEFT"/"RIGHT" (a miss
 * with an unknown along-shot direction: the fit uses it as "did not go in").
 */
public final class ShotCsv {

    private ShotCsv() {
    }

    public static final String HEADER = "shot_id,position_id,rpm,hood_deg,range_in,outcome,miss_in,miss_sigma_in,"
            + "rpm_actual,lateral_in,battery_v,time_s,note,"
            + "alliance,x_in,y_in,heading_deg,vx_ips,vy_ips,omega_dps,turret_cmd,turret_deg,turret_rate_dps,"
            + "p_claim,gate_state,override,protect,est_rel_deg,rpm_cmd,hood_cmd,burst_index,spacing_s";
    public static final int N_COLUMNS = 13 + 19;

    /** Operator labels. */
    public static final int NONE = 0, HIT = 1, SHORT = 2, LONG = 3, LEFT = 4, RIGHT = 5;
    public static final String[] OUTCOME = {"", "hit", "short", "long", "miss", "miss"};
    public static final String[] LABEL = {"unlabelled", "HIT", "SHORT", "LONG", "LEFT", "RIGHT"};

    /** The state at the moment the feed started (one per shot). NaN = not observed (empty cell). */
    public static final class Shot {
        public String shotId = "", positionId = "", alliance = "", gateState = "", note = "";
        public double rpmCmd = Double.NaN, hoodDeg = Double.NaN, rangeIn = Double.NaN, rpmActual = Double.NaN,
                batteryV = Double.NaN, timeS = Double.NaN;
        public double xIn = Double.NaN, yIn = Double.NaN, headingDeg = Double.NaN, vxIps = Double.NaN, vyIps = Double.NaN,
                omegaDps = Double.NaN, turretCmd = Double.NaN, turretDeg = Double.NaN, turretRateDps = Double.NaN,
                pClaim = Double.NaN, estRelDeg = Double.NaN, hoodCmd = Double.NaN, spacingS = Double.NaN;
        public boolean override, protect = true;
        public int burstIndex = 0;
        public int label = NONE;
    }

    private static void num(StringBuilder b, double v, int decimals) {
        if (Double.isNaN(v) || Double.isInfinite(v)) return;
        b.append(String.format(Locale.US, "%." + decimals + "f", v));
    }

    private static void text(StringBuilder b, String s) {
        if (s == null || s.isEmpty()) return;
        boolean quote = s.indexOf(',') >= 0 || s.indexOf('"') >= 0 || s.indexOf('\n') >= 0 || s.indexOf('\r') >= 0;
        if (!quote) {
            b.append(s);
            return;
        }
        b.append('"').append(s.replace("\"", "\"\"").replace("\r", " ").replace("\n", " ")).append('"');
    }

    /** One data line (no trailing newline) for {@link #HEADER}. */
    public static String line(Shot s) {
        StringBuilder b = new StringBuilder(256);
        text(b, s.shotId); b.append(',');
        text(b, s.positionId); b.append(',');
        num(b, s.rpmCmd, 1); b.append(',');
        num(b, s.hoodDeg, 3); b.append(',');
        num(b, s.rangeIn, 2); b.append(',');
        b.append(OUTCOME[s.label]); b.append(',');
        b.append(',');                                  // miss_in: not measured by the tuner
        b.append(',');                                  // miss_sigma_in
        num(b, s.rpmActual, 1); b.append(',');
        b.append(',');                                  // lateral_in
        num(b, s.batteryV, 2); b.append(',');
        num(b, s.timeS, 3); b.append(',');
        StringBuilder note = new StringBuilder();
        if (s.label == LEFT || s.label == RIGHT) note.append(LABEL[s.label]);
        if (s.label == NONE) note.append("unlabelled");
        if (s.override) note.append(note.length() > 0 ? " " : "").append("OVERRIDE");
        if (!s.protect) note.append(note.length() > 0 ? " " : "").append("PROTECTION_OFF");
        if (s.note != null && !s.note.isEmpty()) note.append(note.length() > 0 ? " " : "").append(s.note);
        text(b, note.toString()); b.append(',');
        text(b, s.alliance); b.append(',');
        num(b, s.xIn, 2); b.append(',');
        num(b, s.yIn, 2); b.append(',');
        num(b, s.headingDeg, 2); b.append(',');
        num(b, s.vxIps, 2); b.append(',');
        num(b, s.vyIps, 2); b.append(',');
        num(b, s.omegaDps, 2); b.append(',');
        num(b, s.turretCmd, 3); b.append(',');
        num(b, s.turretDeg, 3); b.append(',');
        num(b, s.turretRateDps, 2); b.append(',');
        num(b, s.pClaim, 4); b.append(',');
        text(b, s.gateState); b.append(',');
        b.append(s.override ? "1" : "0"); b.append(',');
        b.append(s.protect ? "1" : "0"); b.append(',');
        num(b, s.estRelDeg, 3); b.append(',');
        num(b, s.rpmCmd, 1); b.append(',');
        num(b, s.hoodCmd, 3); b.append(',');
        b.append(s.burstIndex); b.append(',');
        num(b, s.spacingS, 3);
        return b.toString();
    }

    /** position_id of a stand: "P45" for a nominal wheel-to-goal range of 45 in. */
    public static String positionId(int nominalRangeIn) {
        return nominalRangeIn > 0 ? "P" + nominalRangeIn : "";
    }
}
