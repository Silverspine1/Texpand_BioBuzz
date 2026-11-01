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
 * Under-shooter shot control (REDUCED targeter, docs/SHOT_ALGO_DESIGN.md sections 7 and 11 in the sim repo).
 * OFF by default: with {@link #enabled} false the shooter runs exactly the old Shotplanner path.
 * Edit live in FTC Dashboard; gamepad2 START in Test_tele saves to {@code /sdcard/FIRST/settings/shot_control.properties}.
 *
 * <p>Model files (copy with adb, see {@code ReducedShotModel}): {@code /sdcard/FIRST/settings/<modelFile>} (required;
 * default {@code robot_model_red_audience.json}) and {@code /sdcard/FIRST/settings/under_shooter_field_tune.json}
 * (optional field-tune export; replaces the shot model). RECOMMENDED model: the R4 export
 * {@code robot_model_red_audience_r4.json} (aim table with P(stay in cell), bearing calibration in occ.f):
 * <pre>
 * adb push outputs/shotcontrol/algos/robot_model_red_audience_r4.json /sdcard/FIRST/settings/
 * </pre>
 * then set {@link #modelFile} to {@code robot_model_red_audience_r4.json} (dashboard), save (gamepad2 START) and reload
 * (gamepad2 X), or push it under the default name.
 */
@Config
public class ShotControlTuning {

    private static final String FILE_NAME = "shot_control.properties";

    /** Master switch for the new rpm / hood / turret command source. false = old behaviour, unchanged. */
    public static boolean enabled = false;
    /** Alliance (the Java pose is one fixed field frame; blue is rotated 180 deg into its team frame). */
    public static boolean blueAlliance = true;
    /** Use the optional field-tune export next to the model when it exists. */
    public static boolean useFieldTune = true;
    /** Model file name in /sdcard/FIRST/settings/ (blank = ReducedShotModel.MODEL_FILE). Recommended:
     *  robot_model_red_audience_r4.json. */
    public static String modelFile = "robot_model_red_audience.json";
    /** Field-tune export in /sdcard/FIRST/settings/ applied over the model's shot block (ShotTuner: gamepad2 right stick
     *  button toggles useFieldTune; change the name here to swap exports, then gamepad2 left stick button reloads). */
    public static String fieldTuneFile = "under_shooter_field_tune.json";

    // Wheel ceiling per battery (R4): rpm_max = min(model rpm_max, fraction x no-load rpm x V / 12). States that need
    // more become infeasible (claim 0) instead of leaving low. No-load 5700 is the sim's flywheel model: MEASURE.
    public static boolean rpmMaxFromBattery = true;
    public static double wheelNoLoadRpmAt12V = 5700;
    public static double rpmMaxNoLoadFraction = 0.98;

    // Turret axis: true = ShooterTuning.turretAxisForwardMeters / turretAxisSidewaysMeters (offset from the
    // odometry point), false = the model file's pivot constants.
    public static boolean pivotFromShooterTuning = true;
    // The model adds turret_lead_s x bearing rate on top of SetTurretAngle's own latency lead (tuned that way in sim).
    public static boolean useModelTurretLead = true;
    // Solve period [ms]: the turret was tuned with a 50 ms command period (old planner); the solve itself is ~us.
    public static double solvePeriodMs = 50;

    // Driver trims (gamepad2 dpad in Test_tele)
    public static double speedTrimPct = 0.0;
    public static double turretTrimDeg = 0.0;
    public static double hoodTrimDeg = 0.0;
    public static double speedTrimStepPct = 0.5;
    public static double turretTrimStepDeg = 0.5;

    // Hood: travel 0..6.3 deg (lower stop = 0) -> servo degrees. 48/40 gears: 1.2 servo deg per hood deg. MEASURE.
    public static double hoodServoAtZeroDeg = 0.0;
    public static double hoodServoDegPerHoodDeg = 1.2;

    // Fire gate (R4 + R5 turret rule, FireGate = fire_gate.py). Defaults = the Python constants; they were fitted in sim
    // on a built scenario set: expect the same shape, not the same numbers (docs/SHOT_ALGO_DESIGN.md 5.2).
    /** MOVEMENT PROTECTION (the settle gate). true (default) = hold the fire trigger until the gate says FIRE;
     *  false = fire as soon as the solve is feasible. ShotTuner button: gamepad1 B. */
    public static boolean gateProtect = true;
    /** When the reduced controller is active in Test_tele / ShotTuner, hold the feed (gamepad1 left bumper) until the
     *  gate says FIRE (or the window ends, or the override button). false = the old behaviour, gate only reports. */
    public static boolean gateHoldsFire = true;
    public static double gateTurRelTolDeg = 6.0;     // refuse: |turret error predicted at release| above this
    public static double gateTurErrDeg = 20.0;       // refuse: |turret cmd - meas| above this (sanity limit of the prediction)
    public static double gateTurRateCapDps = 130.0;  // refuse: |turret speed| above this
    public static double gateTargetGain = 0.0;       // weight of the turret command's rate in the prediction (0 = command already leads)
    public static double gateTReleaseS = 0.080;      // decision -> ball pass-through [s] (assumed = sim fire latency; MEASURE)
    public static double gateRpmTolRpm = 40.0;       // refuse: |wheel cmd - meas| above this (command static)
    public static double gateRpmStaticRps = 400.0;   // wheel command counts as static below this rate [rpm/s]
    public static double gateOmegaMaxDps = 250.0;    // refuse: |yaw rate| above this
    public static double gateLookEps = 0.03;         // wait while a look-ahead claim beats the current one by this
    public static boolean gateLookAhead = true;      // 5 extra solves per solve (horizons 0.04..0.30 s)
    public static double gateWindowS = 1.0;          // driver window after a request (gamepad2 A in Test_tele)

    // Burst bookkeeping (balls 2..n): rpm-dip shot detector OFF until checked against real shots
    public static boolean detectShotsFromRpm = false;
    public static double detectDropFraction = 0.5;
    public static double detectMinDropRpm = 40.0;
    public static double burstResetS = 0.8;

    private static boolean tunable(Field f) {
        int m = f.getModifiers();
        return Modifier.isPublic(m) && Modifier.isStatic(m) && !Modifier.isFinal(m)
                && (f.getType() == double.class || f.getType() == boolean.class || f.getType() == String.class);
    }

    public static void save() throws IOException {
        Properties p = new Properties();
        for (Field f : ShotControlTuning.class.getDeclaredFields()) {
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
            p.store(out, "Shot control tune");
        }
    }

    /** Loads saved values over the defaults (enabled and gateProtect are NOT restored: the new controller always
     *  starts OFF and the movement protection always starts ON). */
    public static int load() {
        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            if (!file.exists()) return 0;
            Properties p = new Properties();
            try (InputStream in = new FileInputStream(file)) {
                p.load(in);
            }
            int applied = 0;
            for (Field f : ShotControlTuning.class.getDeclaredFields()) {
                if (!tunable(f) || f.getName().equals("enabled") || f.getName().equals("gateProtect")) continue;
                String text = p.getProperty(f.getName());
                if (text == null) continue;
                try {
                    if (f.getType() == boolean.class) {
                        f.setBoolean(null, Boolean.parseBoolean(text));
                    } else if (f.getType() == String.class) {
                        f.set(null, text.trim());
                    } else {
                        double value = Double.parseDouble(text);
                        if (Double.isNaN(value) || Double.isInfinite(value)) continue;
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

    /** {@code /sdcard/FIRST/settings/<name>} on the Control Hub. */
    public static File settingsFile(String name) {
        return AppUtil.getInstance().getSettingsFile(name);
    }
}
