package org.firstinspires.ftc.teamcode.CommandBase.NewCode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.IOException;
import java.util.Properties;

/**
 * Shooter, planner and goal numbers. Edit live in FTC Dashboard, then press left-stick click in Test_tele to save;
 * the saved file is loaded whenever the shooter starts. New fields here are saved and loaded automatically.
 */
@Config
public class ShooterTuning {

    private static final String FILE_NAME = "shooter_tune.properties";

    // Bumped when the code defaults must win once: a file saved by an older version is ignored (built-in defaults) until the
    // next save writes this version. Version 3 (2026-11-02): turret axis on the robot centre, model r10.
    private static final String SETTINGS_VERSION = "3";

    // The old Shotplanner's shot physics, hood launch-angle mapping and goal positions are deleted: the reduced model
    // (ShotControlTuning) owns the goal, the hood (hoodServoPosAtZero / AtMax) and the rpm.
    public static double turretAxisForwardMeters = 0.0;
    // (ShotControlTuning) owns the goal, the hood (hoodBottomPos / hoodTopPos) and the rpm.
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
    public static double startXcm = 0;
    public static double startYcm = 0;
    public static double startHeadingDeg = 0;

    private static String loadStatus = "shooter: not loaded yet";
    private static String saveStatus = "shooter: not saved yet";

    static String loadStatus() { return loadStatus; }

    static String saveStatus() { return saveStatus; }

    /** Saves ShooterTuning (shooter_tune.properties); the file is read back and compared. */
    public static int save() throws IOException {
        File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
        int n = TuningIO.save(ShooterTuning.class, file, "Shooter tune", false, SETTINGS_VERSION);
        saveStatus = "shooter: wrote " + n + " values to " + file.getPath() + " (read back OK)";
        return n;
    }

    /** Loads any saved values over the defaults. Returns how many were applied; the details are in ShotControlTuning.settingsLine(). */
    public static int load() {
        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            if (!file.exists()) {
                loadStatus = "shooter: no saved file (" + file.getPath() + ") -> built-in defaults";
                return 0;
            }
            Properties p = TuningIO.read(file);
            if (!SETTINGS_VERSION.equals(p.getProperty("settingsVersion"))) {
                loadStatus = "shooter: saved file is version " + p.getProperty("settingsVersion") + ", not " + SETTINGS_VERSION
                        + " -> IGNORED this run, built-in defaults (save to keep them)";
                return 0;
            }
            TuningIO.Loaded r = TuningIO.load(ShooterTuning.class, p, false, new String[0], new String[0], false);
            loadStatus = TuningIO.describe("shooter", file, r);
            return r.applied;
        } catch (Throwable t) {
            loadStatus = "shooter: LOAD FAILED (" + t + ") -> built-in defaults";
            return 0;
        }
    }
}
