package org.firstinspires.ftc.teamcode.CommandBase.NewCode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.Properties;

/**
 * Under-shooter shot control (REDUCED targeter, docs/SHOT_ALGO_DESIGN.md sections 7 and 11 in the sim repo).
 * The only shot command source (the old Shotplanner is deleted).
 * Edit live in FTC Dashboard; gamepad2 START in Test_tele saves to {@code /sdcard/FIRST/settings/shot_control.properties}.
 *
 * <p>Model: ONE file, {@code robot_model_red_audience_r5.json}, built into the app (TeamCode/src/main/assets/). Nothing on the
 * hub overrides it (no /sdcard model, no field-tune export); to change the model replace that asset and rebuild.
 */
@Config
public class ShotControlTuning {

    private static final String FILE_NAME = "shot_control.properties";

    /** Master switch for the new rpm / hood / turret command source. false = old behaviour, unchanged. */
    /** Alliance (the Java pose is one fixed field frame; blue is rotated 180 deg into its team frame). */
    /** Flywheel motor direction. false = FORWARD (positive power spins the wheel to shoot). The earlier hard-coded REVERSE
     *  (commit 'reverse', old shooter) ran the under-shooter wheel backwards on the first robot test (2026-11-01). Flip it
     *  on the Dashboard if the wheel spins the wrong way; Shot Tuner shows a BACKWARDS warning. */
    public static boolean shooterMotorReversed = false;
    public static boolean blueAlliance = true;

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

    // ===================== THE GOAL (gamepad2 in Shot Tuner and Test_tele, START saves) =====================
    // The goal is the CLOSE (audience-side) cell of your alliance, as a global point in the odometry X / Y frame (origin = the red
    // audience-left corner, +X = audience wall -> back wall, +Y = to the right seen from the audience = toward blue). The field is
    // mirror symmetric across the red-blue axis, so the close cell is red 138.5 / 147.1 cm and blue 138.5 / 211.9 cm. The alliance
    // switch (gamepad2 RIGHT BUMPER) picks which one. Before 2026-11-02 blue was the 180 deg ROTATION of red = the FAR cell, 220.5 / 211.9:
    // 31 in too deep. The trim below is added to that cell (both axes in global cm); the Dashboard and the display show the sum.
    public static double goalTrimXcm = 0.0;
    public static double goalTrimYcm = 0.0;
    // gamepad2 D-pad: UP / DOWN = goal X +- step (away from / toward the audience wall), RIGHT / LEFT = goal Y +- step (toward blue /
    // toward red). LEFT TRIGGER held = the coarse step. BACK = trim back to 0.
    public static double goalStepCm = 1.0;
    public static double goalStepCoarseCm = 5.0;

    // Dashboard only (no buttons any more). rpmScale: real wheel rpm = model rpm x rpmScale (1.0 = the contact sim as built).
    // angleOffsetDeg: the real exit angle minus the model's [deg]. Both were tuned on the wrong goal position before 2026-11-02, so
    // an older saved file does not restore them (settings version 3).
    public static double rpmScale = 1.0;
    public static double angleOffsetDeg = 0.0;
    // Turret angle gain: the commanded turret angle is multiplied by this (1.0 = off); only for a turret that turns less than its encoder says.
    public static double turretScale = 1.0;

    // Field-centric drive is always on: stick UP = away from the driver across the field, whatever the robot's heading.
    // fieldDriveOffsetDeg rotates that stick frame (0 = driver stands at his own alliance wall; + = toward increasing heading).
    public static double fieldDriveOffsetDeg = 0.0;
    // ==========================================================================================================================
    // Turret aim trim (not a model value): hold gamepad2 LEFT TRIGGER + D-pad LEFT / RIGHT.
    public static double turretTrimDeg = 0.0;

    // Hood: travel 0..6.3 deg (lower stop = 0) -> servo degrees. 48/40 gears: 1.2 servo deg per hood deg. MEASURE.
    public static double hoodServoAtZeroDeg = 0.0;
    public static double hoodServoDegPerHoodDeg = 1.2;
    // Hood servo in plain 0..1 servo units, written straight to the servo. BOTTOM = far shots (flattest, hood travel 0),
    // TOP = near shots (steepest, travel 6.3 deg). Measured in the Deez program (same servo, same raw setPosition).
    // (New names, so values saved under older names are not loaded.)
    public static double hoodBottomPos = 0.182;
    public static double hoodTopPos = 0.218;
    // HOOD DEBUG (gamepad2 Y toggles it, Shot Tuner and Test_tele; works like Deez): the hood servo gets hoodDebugPos, the
    // targeting no longer moves the hood. D-pad UP / DOWN = +/- 0.001, D-pad RIGHT = hoodTopPos, D-pad LEFT = hoodBottomPos.
    // The other trim buttons are off while it is on. Never restored from the saved file (always starts OFF).
    public static boolean hoodDebug = false;
    public static double hoodDebugPos = 0.2;

    // Shooting while moving. The wheel and hood need time to reach a new command, so the rpm and hood are solved for where
    // the robot will be mechDelayS from now (its velocity and acceleration). The turret has its own lead (SetTurretAngle). 0 = off.
    public static double mechDelayS = 0.080;
    // Use the robot's acceleration in the shot prediction (estimated from the filtered odometry velocity, clamped).
    public static boolean useAcceleration = true;
    public static double accelFilterS = 0.10;
    public static double accelMaxIps2 = 300.0;

    // Turret hold: while the robot is standing still the solved turret angle is only re-sent when it moves by more than this
    // (the odometry/vision noise in the solve otherwise makes the servos chase +-0.3 deg and the turret oscillates).
    public static double turretHoldBandDeg = 0.7;
    public static double turretHoldMaxSpeedIps = 2.0;
    public static double turretHoldMaxYawDps = 3.0;
    // Turret position gain override (predictive mode kPos, 1/s). 0 = keep the saved tune's value. Lower = softer, less hunting.
    public static double turretKPos = 8.0;

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
    public static double gateAccelHoldIps2 = 100.0;  // hold while |a| or |d speed/dt| of the chassis velocity exceeds this [in/s^2]; 0 = off
    public static double gateLookEps = 0.03;         // wait while a look-ahead claim beats the current one by this
    public static boolean gateLookAhead = true;      // 5 extra solves per solve (horizons 0.04..0.30 s)
    public static double gateWindowS = 1.0;          // driver window after a request (gamepad2 A in Test_tele)

    // Burst bookkeeping (balls 2..n): rpm-dip shot detector OFF until checked against real shots
    public static boolean detectShotsFromRpm = false;
    public static double detectDropFraction = 0.5;
    public static double detectMinDropRpm = 40.0;
    public static double burstResetS = 0.8;

    // Not restored from the saved file: the new controller always starts OFF, the movement protection always starts ON, the hood
    // debug always starts OFF (and its position is only a scratch value).
    private static final String[] NOT_RESTORED = {"enabled", "gateProtect", "hoodDebug", "hoodDebugPos"};
    // Settings version 3 (2026-11-02): the goal is a global point; these were tuned against the old (wrong) blue goal, so a file saved
    // by an older version does not restore them.
    private static final String SETTINGS_VERSION = "4";
    // Version 4 (2026-11-02): back to the r4 model (the r4h hood map was a patch for the wrong goal), so a saved model name is dropped.
    private static final String[] RESET_BEFORE_V3 = {"rpmScale", "angleOffsetDeg", "turretTrimDeg"};
    private static final String[] RESET_BEFORE_V4 = {};

    private static String loadStatus = "shot_control: not loaded yet";
    private static String saveStatus = "nothing saved yet";

    /** Saves ShotControlTuning (shot_control.properties); the file is read back and compared. Returns the count written. */
    public static int save() throws IOException {
        File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
        int n = TuningIO.save(ShotControlTuning.class, file, "Shot control tune", true, SETTINGS_VERSION);
        saveStatus = "shot_control: wrote " + n + " values to " + file.getPath() + " (read back OK)";
        return n;
    }

    /**
     * THE save: ShotControlTuning + ShooterTuning (start pose, turret axis, flywheel gains). Returns the text for the display;
     * it says SAVE FAILED with the reason when either file could not be written or read back.
     */
    public static String saveAll() {
        try {
            ShooterTuning.save();
            save();
            return "SAVED. " + saveStatus + " | " + ShooterTuning.saveStatus();
        } catch (Throwable t) {
            saveStatus = "SAVE FAILED: " + t.getMessage();
            return saveStatus;
        }
    }

    /** What the last load / save did, for the top of every display (so you can see the hub really has your numbers). */
    public static String settingsLine() {
        return loadStatus + " | " + ShooterTuning.loadStatus() + " | last save: " + saveStatus;
    }

    /** Loads saved values over the defaults (see NOT_RESTORED). Returns how many were applied; the details are in settingsLine(). */
    public static int load() {
        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            if (!file.exists()) {
                loadStatus = "shot_control: no saved file (" + file.getPath() + ") -> built-in defaults";
                return 0;
            }
            java.util.Properties saved = TuningIO.read(file);
            String version = saved.getProperty("settingsVersion");
            boolean upgraded = !SETTINGS_VERSION.equals(version);
            String[] reset = "3".equals(version) ? RESET_BEFORE_V4 : RESET_BEFORE_V3;
            TuningIO.Loaded r = TuningIO.load(ShotControlTuning.class, saved, true, NOT_RESTORED, reset, upgraded);
            loadStatus = TuningIO.describe("shot_control", file, r)
                    + (upgraded ? "  (old file: " + Arrays.toString(reset) + " reset to the defaults)" : "");
            return r.applied;
        } catch (Throwable t) {
            loadStatus = "shot_control: LOAD FAILED (" + t + ") -> built-in defaults";
            return 0;
        }
    }

    /** {@code /sdcard/FIRST/settings/<name>} on the Control Hub. */
    /**
     * The tuning buttons, the same in every OpMode (gamepad2): D-pad UP / DOWN = goal X, RIGHT / LEFT = goal Y (LEFT TRIGGER held =
     * the coarse step), BACK = goal back to the defaults, Y = hood debug on / off (then the D-pad moves the hood servo instead).
     * Returns a status text or null.
     */
    public static String trimButtons(com.qualcomm.robotcore.hardware.Gamepad cur, com.qualcomm.robotcore.hardware.Gamepad last) {
        if (cur.y && !last.y) {                        // HOOD DEBUG on / off
            hoodDebug = !hoodDebug;
            if (hoodDebug) hoodDebugPos = hoodBottomPos;
        }
        if (hoodDebug) {                               // the goal buttons are off while it is on
            if (cur.dpad_up && !last.dpad_up) hoodDebugPos += 0.001;
            if (cur.dpad_down && !last.dpad_down) hoodDebugPos -= 0.001;
            if (cur.dpad_right && !last.dpad_right) hoodDebugPos = hoodTopPos;
            if (cur.dpad_left && !last.dpad_left) hoodDebugPos = hoodBottomPos;
            hoodDebugPos = Math.max(0.0, Math.min(1.0, Math.round(hoodDebugPos * 1000.0) / 1000.0));
            return String.format(java.util.Locale.US, "HOOD DEBUG ON: servo %.3f  (D-pad up/down +-0.001, right = top %.3f, left = bottom %.3f, gamepad2 Y = off)",
                    hoodDebugPos, hoodTopPos, hoodBottomPos);
        }
        final double step = cur.left_trigger > 0.5 ? goalStepCoarseCm : goalStepCm;
        String s = null;
        if (cur.dpad_up && !last.dpad_up) { goalTrimXcm = round1(goalTrimXcm + step); s = "goal trim X " + goalTrimXcm; }
        if (cur.dpad_down && !last.dpad_down) { goalTrimXcm = round1(goalTrimXcm - step); s = "goal trim X " + goalTrimXcm; }
        if (cur.dpad_right && !last.dpad_right) { goalTrimYcm = round1(goalTrimYcm + step); s = "goal trim Y " + goalTrimYcm; }
        if (cur.dpad_left && !last.dpad_left) { goalTrimYcm = round1(goalTrimYcm - step); s = "goal trim Y " + goalTrimYcm; }
        if (cur.back && !last.back) { goalTrimXcm = 0.0; goalTrimYcm = 0.0; s = "goal trim reset to 0"; }
        return s;
    }

    /** Hood ends and the check buttons, for every display. */
    public static String hoodLine() {
        return String.format(java.util.Locale.US, "bottom/far %.3f  top/near %.3f   (gamepad2 Y = HOOD DEBUG on/off)",
                hoodBottomPos, hoodTopPos);
    }

    /** The Dashboard-only values, for the detailed display. */
    public static String tuneLine() {
        return String.format(java.util.Locale.US, "rpmScale %.2f   angleOffset %+.1f deg   turret trim %+.1f deg   turretScale %.3f   (Dashboard)",
                rpmScale, angleOffsetDeg, turretTrimDeg, turretScale);
    }

    private static double round1(double v) {
        return Math.round(v * 10.0) / 10.0;
    }

    /** Null when the saved settings loaded and saved fine, else what is wrong (for the warning line). */
    public static String settingsProblem() {
        String all = loadStatus + " | " + ShooterTuning.loadStatus() + " | " + saveStatus;
        return all.contains("FAILED") || all.contains("UNREADABLE") ? all : null;
    }

    public static File settingsFile(String name) {
        return AppUtil.getInstance().getSettingsFile(name);
    }
}
