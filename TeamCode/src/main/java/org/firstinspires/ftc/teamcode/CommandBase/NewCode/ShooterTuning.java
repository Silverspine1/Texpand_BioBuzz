package org.firstinspires.ftc.teamcode.CommandBase.NewCode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

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
 * Shooter, planner and goal numbers. Edit live in FTC Dashboard, then press left-stick click in Test_tele to save;
 * the saved file is loaded whenever the shooter starts. New fields here are saved and loaded automatically.
 */
@Config
public class ShooterTuning {

    private static final String FILE_NAME = "shooter_tune.properties";

    // Shot physics
    public static double flywheelRadiusMeters = 0.036;
    public static double muzzleEfficiency = 1.0;
    public static double magnusK = 0.0065;
    public static double maxFlywheelRPM = 6000;
    public static double requiredClearanceMeters = 0.025;
    public static double muzzleHeightMeters = 0.2;
    public static double turretAxisForwardMeters = 0.1;
    public static double muzzleForwardFromTurretMeters = 0.05;
    public static double plannerDelaySeconds = 0.10;
    public static double plannerPeriodMs = 50;

    // Hood: two measured points (servo degrees -> launch angle above horizontal) define the whole mapping.
    // The hood is 15-45 deg from vertical, so the launch angle is 90 minus that: 75 down to 45.
    public static double hoodServoAtMinDeg = 0;
    public static double hoodServoAtMaxDeg = 253;
    public static double hoodLaunchAtMinDeg = 75;
    public static double hoodLaunchAtMaxDeg = 45;

    // Goal, field metres
    public static double audienceGoalX = 1.22;
    public static double audienceGoalY = 2.15;
    public static double oppositeGoalX = 2.44;
    public static double oppositeGoalY = 2.15;
    public static double openingBottomZ = 1.36;

    // Flywheel: power = kF * targetRPM + kP * error + integral(kI * error)
    public static double flywheelTicksPerRev = 33.6;
    public static double flywheelKF = 0.00018;
    public static double flywheelKP = 0.0003;
    public static double flywheelKI = 0.0008;
    public static double flywheelIntegralMax = 0.25;
    public static double flywheelIntegralZoneRpm = 300;
    public static double flywheelMaxPower = 1.0;
    public static double flywheelRpmFilter = 0.3;
    public static double flywheelReadyPercent = 3.0;

    // Start pose used by Test_tele at init and by its set-pose button (cm, degrees)
    public static double startXcm = 0;
    public static double startYcm = 0;
    public static double startHeadingDeg = 0;

    // Dashboard field view
    public static boolean fieldView = true;
    public static double fieldViewRotationDeg = 0;
    public static boolean fieldViewFlipY = false;
    public static double mapCellMeters = 0.3;
    public static double robotSizeMeters = 0.4;

    private static boolean tunable(Field f) {
        int m = f.getModifiers();
        return Modifier.isPublic(m) && Modifier.isStatic(m) && !Modifier.isFinal(m)
                && (f.getType() == double.class || f.getType() == boolean.class);
    }

    public static void save() throws IOException {
        Properties p = new Properties();
        for (Field f : ShooterTuning.class.getDeclaredFields()) {
            if (!tunable(f)) continue;
            try {
                p.setProperty(f.getName(), String.valueOf(f.get(null)));
            } catch (IllegalAccessException ignored) {
            }
        }

        File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
        File parent = file.getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs()) {
            throw new IOException("Could not create " + parent);
        }
        try (OutputStream out = new FileOutputStream(file)) {
            p.store(out, "Shooter tune");
        }
    }

    /** Loads any saved values over the defaults. Returns how many were applied. */
    public static int load() {
        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            if (!file.exists()) return 0;

            Properties p = new Properties();
            try (InputStream in = new FileInputStream(file)) {
                p.load(in);
            }

            int applied = 0;
            for (Field f : ShooterTuning.class.getDeclaredFields()) {
                if (!tunable(f)) continue;
                String text = p.getProperty(f.getName());
                if (text == null) continue;
                try {
                    if (f.getType() == boolean.class) {
                        f.setBoolean(null, Boolean.parseBoolean(text));
                    } else {
                        double value = Double.parseDouble(text);
                        if (!Double.isFinite(value)) continue;
                        f.setDouble(null, value);
                    }
                    applied++;
                } catch (IllegalAccessException | NumberFormatException ignored) {
                }
            }
            return applied;
        } catch (Throwable t) {
            return 0;
        }
    }
}
