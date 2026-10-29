package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Properties;

/**
 * Global pathing values - every Follower / PathsManager built with
 * {@code new RobotConfig()} starts from these, and DriveBase reads its motor
 * protection values from here.
 *
 * Set them either way:
 *   - by hand: edit the defaults below (and set USE_TUNED_FILE = false, or
 *     delete the file, so a saved tune does not override them), or
 *   - with PathTuner: it measures / tunes the drive model, end gains, holds,
 *     overshoots and POWER_LIMIT and saves them to FILE, which is loaded over
 *     these defaults the first time any pathing code runs. Everything else
 *     here (margins, corner accel, motor protection, ...) is never saved, so
 *     editing it in code always takes effect.
 *
 * Units: cm, deg, s, motor power (0..1). Robot frame: FWD = forward,
 * STR = strafe right, TURN = clockwise.
 */
public final class PathingValues {

    public static boolean USE_TUNED_FILE = true;
    public static final String FILE = "/sdcard/FIRST/pathing_values.properties";

    // ---- drive model (PathTuner measures these) ----
    public static double SPEED_PER_POWER_FWD = 260;    // cm/s per unit power above breakaway
    public static double SPEED_PER_POWER_STR = 200;    // cm/s per unit power above breakaway
    public static double SPEED_PER_POWER_TURN = 600;   // deg/s per unit power above breakaway
    public static double TAU_FWD = 0.15;               // speed response time constant, s
    public static double TAU_STR = 0.15;
    public static double TAU_TURN = 0.12;
    public static double DELAY = 0.03;                 // command -> motion dead time, s
    public static double DELAY_TURN = 0.03;
    public static double KS_FWD = 0.06;                // breakaway power
    public static double KS_STR = 0.08;
    public static double KS_TURN = 0.08;
    public static double BRAKE_DECEL_FWD = 900;        // cm/s^2 the protected drive can brake at
    public static double BRAKE_DECEL_STR = 800;

    // ---- motion profile ----
    public static double POWER_LIMIT = 1.0;            // max translation power
    public static double DECEL_MARGIN = 0.85;          // plan with this fraction of BRAKE_DECEL
    public static double OVERSHOOT = 3.0;              // cm - planned stop point past the target
    public static double K_END_FWD = 18;               // 1/s - final approach stiffness
    public static double K_END_STR = 18;
    public static double LAT_ACCEL = 600;              // cm/s^2 - corner speed limit on curves

    // ---- feedback ----
    public static double VEL_GAIN_FWD = 1.0;           // speed error feedback (1 = full-speed error -> 1 power)
    public static double VEL_GAIN_STR = 1.0;
    public static double K_CROSS = 8;                  // 1/s - pull back onto the path

    // ---- end holds ----
    public static double HOLD_T = 2.5;                 // cm - translation output 0 inside this
    public static double HOLD_H = 1.25;                // deg - turn output 0 inside this

    // ---- heading profile (same shape as translation) ----
    public static double BRAKE_DECEL_TURN = 600;       // deg/s^2 the protected drive can stop a turn at
    public static double K_END_H = 10;                 // 1/s - heading final approach stiffness
    public static double VEL_GAIN_H = 1.0;             // turn-rate error feedback
    public static double OVERSHOOT_H = 2.0;            // deg - planned overshoot past the target heading

    // ---- drive side balance (PathTuner measures these) ----
    // A side that pushes harder makes the robot yaw when it should drive
    // straight. Measured from the yaw during the full-power pushes and
    // cancelled per wheel in DriveBase; each wheel is changed by at most
    // MAX_SIDE_TRIM. + = left side stronger / front pair stronger.
    public static double SIDE_TRIM_LR = 0;
    public static double SIDE_TRIM_FB = 0;
    public static final double MAX_SIDE_TRIM = 0.10;

    // ---- DriveBase bare-motor protection ----
    public static double BRAKE_CURRENT_MAX = 0.8;      // braking current cap, fraction of stall
    public static double BRAKE_CURRENT_RATE = 15;      // braking current ramp, stall fractions / s

    // what PathTuner owns - only these are saved / loaded, so hand edits to
    // everything else (motor protection, margins, overshoot, ...) always apply
    private static final java.util.Set<String> TUNED = new java.util.HashSet<>(java.util.Arrays.asList(
            "SPEED_PER_POWER_FWD", "SPEED_PER_POWER_STR", "SPEED_PER_POWER_TURN",
            "TAU_FWD", "TAU_STR", "TAU_TURN", "DELAY", "DELAY_TURN",
            "KS_FWD", "KS_STR", "KS_TURN", "BRAKE_DECEL_FWD", "BRAKE_DECEL_STR",
            "POWER_LIMIT", "K_END_FWD", "K_END_STR", "VEL_GAIN_FWD", "VEL_GAIN_STR",
            "HOLD_T", "HOLD_H", "SIDE_TRIM_LR", "SIDE_TRIM_FB", "OVERSHOOT",
            "BRAKE_DECEL_TURN", "K_END_H", "VEL_GAIN_H", "OVERSHOOT_H"));

    private static boolean loaded = false;

    private PathingValues() {}

    /** Loads FILE once (if USE_TUNED_FILE and it exists). Safe to call often. */
    public static synchronized void ensureLoaded() {
        if (loaded) return;
        if (!USE_TUNED_FILE) { loaded = true; return; }
        File f = new File(FILE);
        if (!f.exists()) { loaded = true; return; }
        Properties p = new Properties();
        try (InputStream in = new FileInputStream(f)) {
            p.load(in);
        } catch (IOException e) {
            return;     // retried next call
        }
        loaded = true;
        for (Field fd : values()) {
            if (!TUNED.contains(fd.getName())) continue;
            String v = p.getProperty(fd.getName());
            if (v == null) continue;
            try {
                double d = Double.parseDouble(v.trim());
                boolean signed = fd.getName().startsWith("SIDE_TRIM");
                if (!Double.isNaN(d) && !Double.isInfinite(d) && (signed || d >= 0)) fd.setDouble(null, d);
            } catch (Exception ignored) {}
        }
    }

    /** Writes the tuned values to FILE. */
    public static synchronized boolean save() {
        Properties p = new Properties();
        for (Field fd : values()) {
            if (!TUNED.contains(fd.getName())) continue;
            try {
                p.setProperty(fd.getName(), Double.toString(fd.getDouble(null)));
            } catch (IllegalAccessException ignored) {}
        }
        File f = new File(FILE);
        File dir = f.getParentFile();
        if (dir != null && !dir.exists()) dir.mkdirs();
        try (OutputStream out = new FileOutputStream(f)) {
            p.store(out, "PathingValues - written by PathTuner");
            return true;
        } catch (IOException e) {
            return false;
        }
    }

    /** Java source for the current values, to paste over the defaults. */
    public static String asJava() {
        StringBuilder sb = new StringBuilder();
        for (Field fd : values()) {
            try {
                sb.append(String.format("    public static double %s = %s;\n", fd.getName(), fmt(fd.getDouble(null))));
            } catch (IllegalAccessException ignored) {}
        }
        return sb.toString();
    }

    private static String fmt(double v) {
        if (v == Math.rint(v) && Math.abs(v) < 1e6) return String.valueOf((long) v);
        return String.format("%.5g", v);
    }

    private static java.util.List<Field> values() {
        java.util.List<Field> out = new java.util.ArrayList<>();
        for (Field fd : PathingValues.class.getDeclaredFields()) {
            int m = fd.getModifiers();
            if (Modifier.isPublic(m) && Modifier.isStatic(m) && !Modifier.isFinal(m) && fd.getType() == double.class) {
                out.add(fd);
            }
        }
        return out;
    }
}
