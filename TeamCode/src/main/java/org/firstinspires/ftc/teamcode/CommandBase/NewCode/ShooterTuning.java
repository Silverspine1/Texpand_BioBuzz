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

    // Bumped when defaults change meaning; values listed in RESET_ON_UPGRADE are then taken from the new defaults.
    private static final String SETTINGS_VERSION = "2";
    private static final String[] RESET_ON_UPGRADE = {"flywheelTicksPerRev", "flywheelKF"};

    // The old Shotplanner's shot physics, hood launch-angle mapping and goal positions are deleted: the reduced model
    // (ShotControlTuning) owns the goal, the hood (hoodServoPosAtZero / AtMax) and the rpm.
    public static double turretAxisForwardMeters = 0.05;
    // Sideways offset of the turret axis from the odometry point. Positive is toward the side heading increases toward.
    public static double turretAxisSidewaysMeters = 0.0;
    // true = the program ASSUMES the turret points exactly straight forward when it starts (INIT and again at PLAY): the encoder
    // is zeroed there and turretZeroOffsetDeg is forced to 0. false = use turretZeroOffsetDeg as a saved calibration.
    public static boolean assumeTurretForwardAtStart = true;
    // Turret encoder reading when the turret really points straight ahead (only used when assumeTurretForwardAtStart is false)
    public static double turretZeroOffsetDeg = 0.0;

    // Motion input cleaning for the shot controller: velocities are smoothed, acceleration is off (differentiated noise)
    public static double plannerVelocityFilterSeconds = 0.06;
    public static double plannerVelocityDeadbandMps = 0.02;
    public static double plannerHeadingRateDeadbandDps = 2.0;
    public static double plannerTurretRateDeadbandDps = 5.0;
    public static boolean plannerUseAcceleration = false;

    // Flywheel: power = kF * targetRPM + kP * error + integral(kI * error)
    // 1:1 goBILDA 6000 rpm motor is 28 ticks per rev. Check: full power with no load should read about 5800-6000 rpm.
    public static double flywheelTicksPerRev = 28;
    public static double flywheelKF = 0.00018;
    public static double flywheelKP = 0.0003;
    public static double flywheelKI = 0.0008;
    public static double flywheelIntegralMax = 0.25;
    public static double flywheelIntegralZoneRpm = 300;
    public static double flywheelMaxPower = 1.0;
    public static double flywheelRpmFilter = 0.3;
    public static double flywheelReadyPercent = 3.0;

    // Start pose used by Test_tele / Shot Tuner at init and by their set-pose button (cm, degrees)
    public static double startXcm = 20;
    public static double startYcm = 344;
    public static double startHeadingDeg = 0;

    private static boolean tunable(Field f) {
        int m = f.getModifiers();
        return Modifier.isPublic(m) && Modifier.isStatic(m) && !Modifier.isFinal(m)
                && (f.getType() == double.class || f.getType() == boolean.class);
    }

    public static void save() throws IOException {
        Properties p = new Properties();
        p.setProperty("settingsVersion", SETTINGS_VERSION);
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

            boolean upgraded = !SETTINGS_VERSION.equals(p.getProperty("settingsVersion"));
            int applied = 0;
            for (Field f : ShooterTuning.class.getDeclaredFields()) {
                if (!tunable(f)) continue;
                if (upgraded && java.util.Arrays.asList(RESET_ON_UPGRADE).contains(f.getName())) continue;
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
