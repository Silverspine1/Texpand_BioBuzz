package org.firstinspires.ftc.teamcode.CommandBase.NewCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Intake.intakeHardware_doubleBot;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.ReducedShotController;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.ShotCampaign;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.ShotCsv;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.ShotHold;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.ShooterController.TurretDebugMode;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * SHOT TUNER - runs the REDUCED shot controller on the real robot, holds the feed until the fire gate says FIRE, and
 * logs every shot as one line of the field-tune CSV (docs/SHOT_CONTROL_README.md in the sim repo: pull the file, run
 * {@code python -m ftc_optimizer.under_shooter.field_tune fit}). Works from the two gamepads alone; everything also
 * shows on the FTC Dashboard (Configuration > ShotControlTuning, plus a telemetry copy).
 *
 * <p>Starts with the reduced controller ON, targeting ON and MOVEMENT PROTECTION ON. Power the robot with the turret
 * pointing straight forward (the encoder zeroes at INIT) and put the model files on the hub first
 * ({@code ReducedShotModel}). HARDWARE / OpMode glue is compile-checked only; the pure parts are in
 * {@code ShotHold}, {@code ShotCsv}, {@code ShotCampaign}, {@code FireGate} (tools/shot_java_parity/ShotTunerTest.java).
 *
 * <pre>
 * gamepad1 (driver)
 *   right stick / triggers   drive and turn (as Test_tele)       right bumper   intake
 *   LEFT BUMPER              FEED / FIRE. With protection ON it is HELD until the gate says FIRE (or the window of
 *                            ShotControlTuning.gateWindowS runs out and the solve is feasible). One press = one shot row.
 *   B                        MOVEMENT PROTECTION ON / OFF (the settle gate); also the tick box gateProtect on the Dashboard
 *   A (hold)                 manual OVERRIDE: feed now, whatever the gate says
 *   Y / X                    campaign: skip this step / go back one step
 *   D-pad up                 targeting on / off          right stick button   set the start pose (ShooterTuning)
 * gamepad2 (operator): label the shot you just took
 *   A HIT   B SHORT   X LONG   LEFT BUMPER LEFT   RIGHT BUMPER RIGHT   (the row is written when you label it)
 *   D-pad up / down   rpmScale +/-     D-pad right / left   angleOffsetDeg +/-     LEFT TRIGGER + D-pad left / right   turret trim
 *   BACK   clear turret trim     Y   HOOD DEBUG on/off: D-pad up/down +-0.001, right = top, left = bottom
 *   START                    save ShotControlTuning to /sdcard/FIRST/settings/shot_control.properties
 *   left stick button        reload the model + field-tune export     right stick button   toggle useFieldTune + reload
 *                            (swap to another export: set ShotControlTuning.fieldTuneFile on the Dashboard, then reload)
 * </pre>
 */
@Config
@TeleOp(name = "Shot Tuner", group = "Tuning")
public class ShotTuner extends OpModeEX {

    /** Folder of the shot logs on the Control Hub (adb pull /sdcard/FIRST/shot_logs). */
    public static String LOG_DIR = "/sdcard/FIRST/shot_logs";
    /** Walk the 4-stand campaign (docs/UNDER_SHOOTER_TUNING.md section 6); false = free shooting, no position ids. */
    public static boolean CAMPAIGN_ON = false;

    private final ShotHold hold = new ShotHold();
    private final ShotCampaign campaign = new ShotCampaign();
    private Telemetry dash;

    private ShotCsv.Shot pending;
    private int shotCount = 0;
    private int lastLabel = ShotCsv.NONE;
    private String status = "";
    private String logPath = "";
    private FileWriter logWriter;
    private String logStatus = "log not opened";
    private boolean overrideNow;
    private boolean flyTest = false, flyTestPrevTargeting = false;
    private boolean initAllianceLatch = false;

    /** INIT loop: keeps odometry running so the start pose is written and shown BEFORE you press PLAY (nothing else runs). */
    @Override
    public void init_loop() {
        odometry.execute();
        telemetry.addLine("=== SHOT TUNER: INIT (not started) ===");
        telemetry.addLine("Check the pose below equals your start pose, the model is loaded, then press PLAY.");
        if (gamepad2.right_bumper && !initAllianceLatch) ShotControlTuning.blueAlliance = !ShotControlTuning.blueAlliance;
        initAllianceLatch = gamepad2.right_bumper;
        put("ALLIANCE", (ShotControlTuning.blueAlliance ? "BLUE" : "RED") + "  (gamepad2 RIGHT BUMPER switches it here in INIT; START saves after PLAY)");
        ShotDebug.pose(odometry, shooterController, this::put);
        put("model", shooterController.reducedStatus);
        put("SETTINGS", ShotControlTuning.settingsLine());
        put("battery", f("%.2f V", shooterController.batteryVolts));
        put("log", logStatus);
        dash.update();
    }

    @Override
    public void initEX() {
        dash = FtcDashboard.getInstance().getTelemetry();
        ShotControlTuning.gateProtect = true;      // protection always starts ON
        if (!shooterController.reduced.hasModel()) shooterController.loadReducedModel();
        shooterController.targeting = true;
        shooterController.turretDebugMode = TurretDebugMode.OFF;
        odometry.setPose(ShooterTuning.startXcm, ShooterTuning.startYcm, ShooterTuning.startHeadingDeg);
        openLog();
        say("Shot Tuner ready. " + shooterController.reducedStatus);
    }

    @Override
    public void loopEX() {
        final ReducedShotController r = shooterController.reduced;
        final double now = shooterController.reducedSeconds();

        // ---- flywheel scale / direction test: gamepad2 RIGHT TRIGGER = that much power, targeting off while held ----
        double trig = gamepad2.right_trigger;
        if (trig > 0.05) {
            if (!flyTest) {
                flyTest = true;
                flyTestPrevTargeting = shooterController.targeting;
            }
            shooterController.targeting = false;
            shooterController.manualFlywheelPower = trig;
        } else if (flyTest) {
            flyTest = false;
            shooterController.manualFlywheelPower = 0.0;
            shooterController.targeting = flyTestPrevTargeting;
        }

        // ---- driver ----
        // stick UP = away from the driver across the field (field-centric, gamepad1 BACK toggles) or the robot's own axes
        if (!lastGamepad1.back && currentGamepad1.back) ShotControlTuning.fieldCentricDrive = !ShotControlTuning.fieldCentricDrive;
        if (ShotControlTuning.fieldCentricDrive) {
            driveBase.driveFieldCentric(-gamepad1.right_stick_y, gamepad1.right_stick_x, (gamepad1.left_trigger - gamepad1.right_trigger),
                    odometry.Heading(), ShotControlTuning.blueAlliance, ShotControlTuning.fieldDriveOffsetDeg);
        } else {
            driveBase.drivePowers(gamepad1.right_stick_y, -gamepad1.right_stick_x, (gamepad1.left_trigger - gamepad1.right_trigger));
        }
        IntakeHardware.Vertical = driveBase.vertical;
        if (!lastGamepad1.dpad_up && currentGamepad1.dpad_up && !flyTest) shooterController.targeting = !shooterController.targeting;
        if (!lastGamepad1.right_stick_button && currentGamepad1.right_stick_button) {
            odometry.setPose(ShooterTuning.startXcm, ShooterTuning.startYcm, ShooterTuning.startHeadingDeg);
            status = "pose set";
        }
        if (!lastGamepad1.b && currentGamepad1.b) ShotControlTuning.gateProtect = !ShotControlTuning.gateProtect;
        if (CAMPAIGN_ON) {
            if (!lastGamepad1.y && currentGamepad1.y) campaign.skip();
            if (!lastGamepad1.x && currentGamepad1.x) campaign.back();
        }
        boolean slip = CAMPAIGN_ON && campaign.isSlip();
        shooterController.forceRpm = slip ? ShotCampaign.SLIP_RPM : 0.0;
        shooterController.forceHoodDeg = slip ? ShotCampaign.SLIP_HOOD_DEG : Double.NaN;
        overrideNow = gamepad1.a || slip;          // the slip shot's rpm is not the solve's: the gate would refuse it

        // ---- feed: the gate's decision held against the driver's trigger ----
        boolean applicable = ShotControlTuning.gateHoldsFire && shooterController.reducedActive && r.hasModel();
        boolean feed = hold.update(now, gamepad1.left_bumper, overrideNow, applicable, r.gateState);
        if (hold.consumeRequestOpened()) shooterController.requestReducedShot();
        if (feed) {
            IntakeHardware.State = intakeHardware_doubleBot.intakeState.transfering;
        } else if (gamepad1.right_bumper) {
            IntakeHardware.State = intakeHardware_doubleBot.intakeState.intaking;
        } else {
            IntakeHardware.State = intakeHardware_doubleBot.intakeState.off;
        }
        if (hold.feedStarted) onShotStarted(r);

        // ---- operator: label, trims, files ----
        if (!lastGamepad2.a && currentGamepad2.a) label(ShotCsv.HIT);
        if (!lastGamepad2.b && currentGamepad2.b) label(ShotCsv.SHORT);
        if (!lastGamepad2.x && currentGamepad2.x) label(ShotCsv.LONG);
        if (!lastGamepad2.left_bumper && currentGamepad2.left_bumper) label(ShotCsv.LEFT);
        if (!lastGamepad2.right_bumper && currentGamepad2.right_bumper) label(ShotCsv.RIGHT);
        String trim = ShotControlTuning.trimButtons(currentGamepad2, lastGamepad2);
        if (trim != null) status = trim + "  (gamepad2 START saves)";
        if (!lastGamepad2.left_stick_button && currentGamepad2.left_stick_button) {
            shooterController.loadReducedModel();
            status = "reloaded: " + shooterController.reducedStatus;
        }
        if (!lastGamepad2.right_stick_button && currentGamepad2.right_stick_button) {
            ShotControlTuning.useFieldTune = !ShotControlTuning.useFieldTune;
            shooterController.loadReducedModel();
            status = "field-tune " + (ShotControlTuning.useFieldTune ? "ON " : "OFF ") + ShotControlTuning.fieldTuneFile
                    + " -> " + shooterController.reducedStatus;
        }
        if (!lastGamepad2.start && currentGamepad2.start) status = ShotControlTuning.saveAll();

        telemetryBlock(r, now, applicable);
        dash.update();
    }

    @Override
    public void stop() {
        if (pending != null) writePending();           // an unlabelled last shot is still a row
        shooterController.forceRpm = 0.0;
        shooterController.forceHoodDeg = Double.NaN;
        closeLog();
        super.stop();
    }

    // ------------------------------------------------------------------ shots and the log

    private void onShotStarted(ReducedShotController r) {
        if (pending != null) writePending();            // the previous shot was never labelled
        shooterController.notifyReducedShot();
        ShotCsv.Shot s = new ShotCsv.Shot();
        shotCount++;
        s.shotId = "tune-" + shotCount;
        s.positionId = CAMPAIGN_ON ? campaign.positionId() : "";
        s.rpmCmd = shooterController.forceRpm > 0.0 ? shooterController.forceRpm : r.rpmCmd;
        s.hoodDeg = Double.isNaN(shooterController.forceHoodDeg) ? r.hoodDeg : shooterController.forceHoodDeg;
        s.rangeIn = r.result.d;                         // turret axis -> aim point (field_tune subtracts the 4 in wheel offset)
        s.rpmActual = shooterController.RPM;
        s.batteryV = shooterController.batteryVolts > 0.0 ? shooterController.batteryVolts : Double.NaN;
        s.timeS = getRuntime();
        s.alliance = ShotControlTuning.blueAlliance ? "blue" : "red";
        s.xIn = r.teamX;
        s.yIn = r.teamY;
        s.headingDeg = r.teamHeading;
        s.vxIps = r.teamVx;
        s.vyIps = r.teamVy;
        s.omegaDps = odometry.getHeadingVelocityDeg();
        s.turretCmd = r.turretJavaDeg;
        s.turretDeg = shooterController.getTurretAimDeg();
        s.turretRateDps = shooterController.encoder.getVelocity();
        s.pClaim = r.feasible ? r.pMake : 0.0;
        s.gateState = ReducedShotController.GATE_TEXT[r.gateState];
        s.override = hold.state == ShotHold.OVERRIDE;
        // rpm in the row is the REAL wheel rpm; the model rpm = rpm / rpmScale (field_tune needs that)
        s.note = f("rpmScale=%.3f angleOffsetDeg=%.2f", ShotControlTuning.rpmScale, ShotControlTuning.angleOffsetDeg);
        s.protect = ShotControlTuning.gateProtect;
        s.estRelDeg = r.gateEstRelDeg;
        s.hoodCmd = r.hoodDeg;
        s.burstIndex = r.shotIndex;
        pending = s;
        status = "shot " + shotCount + " taken: label it (gamepad2 A HIT / B SHORT / X LONG / LB LEFT / RB RIGHT)";
    }

    private void label(int label) {
        if (pending == null) {
            status = "nothing to label yet (fire a shot first)";
            return;
        }
        pending.label = label;
        lastLabel = label;
        writePending();
        if (CAMPAIGN_ON) campaign.shotLabelled();
        status = "shot " + shotCount + " = " + ShotCsv.LABEL[label];
    }

    private void writePending() {
        ShotCsv.Shot s = pending;
        pending = null;
        if (logWriter == null) {
            logStatus = "NOT LOGGED (no file): " + ShotCsv.line(s);
            return;
        }
        try {
            logWriter.write(ShotCsv.line(s));
            logWriter.write("\n");
            logWriter.flush();
            logStatus = "logged " + s.shotId + " to " + logPath;
        } catch (IOException e) {
            logStatus = "LOG WRITE FAILED: " + e.getMessage();
        }
    }

    private void openLog() {
        try {
            File dir = new File(LOG_DIR);
            if (!dir.exists() && !dir.mkdirs()) throw new IOException("cannot create " + dir);
            String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            File f = new File(dir, "shot_tune_" + stamp + ".csv");
            logWriter = new FileWriter(f, false);
            logWriter.write(ShotCsv.HEADER + "\n");
            logWriter.flush();
            logPath = f.getPath();
            logStatus = "logging to " + logPath;
        } catch (IOException e) {
            logWriter = null;
            logStatus = "LOG NOT OPENED: " + e.getMessage();
        }
    }

    private void closeLog() {
        try {
            if (logWriter != null) logWriter.close();
        } catch (IOException ignored) {
        }
        logWriter = null;
    }

    // ------------------------------------------------------------------ telemetry (driver station + dashboard)

    private void say(String text) {
        telemetry.addLine(text);
        dash.addLine(text);
    }

    private void put(String caption, String value) {
        telemetry.addData(caption, value);
        dash.addData(caption, value);
    }

    private static String f(String fmt, Object... a) {
        return String.format(Locale.US, fmt, a);
    }

    private void telemetryBlock(ReducedShotController r, double now, boolean applicable) {
        put("HOOD", shooterController.hoodStatusLine());
        put("SETTINGS", ShotControlTuning.settingsLine());
        put("DRIVE", ShotControlTuning.fieldCentricDrive ? "FIELD-CENTRIC (gamepad1 BACK = robot-centric)" : "ROBOT-CENTRIC (gamepad1 BACK = field-centric)");
        say("TUNE: " + ShotControlTuning.tuneLine());
        say(ShotControlTuning.gateProtect
                ? "=== MOVEMENT PROTECTION: ON  (gamepad1 B turns it OFF) ==="
                : "!!! MOVEMENT PROTECTION: OFF  (fires as soon as feasible; gamepad1 B turns it ON) !!!");
        if (flyTest) say(">>> FLYWHEEL TEST: gamepad2 right trigger held = manual power, targeting OFF. Read the rpm below, then release. <<<");
        ShotDebug.pose(odometry, shooterController, this::put);
        ShotDebug.flywheel(shooterController, this::put);
        say("--- SHOT ---");
        put("feed", hold.state == ShotHold.HOLDING
                ? f("HELD %.2f s: gate says %s%s", hold.heldBackS(now), ReducedShotController.GATE_TEXT[r.gateState],
                    r.gateState == ReducedShotController.GATE_UNSETTLED ? " (" + ReducedShotController.REFUSE_TEXT[r.gateRefuse] + ")" : "")
                : ShotHold.TEXT[hold.state] + (applicable ? "" : "  [gate not holding: controller off / no model / gateHoldsFire off]"));
        put("gate now", ReducedShotController.GATE_TEXT[r.gateState]
                + (r.gateState == ReducedShotController.GATE_UNSETTLED ? " (" + ReducedShotController.REFUSE_TEXT[r.gateRefuse] + ")" : "")
                + (r.requestActive ? (r.gateDeadline ? "  DEADLINE" : f("  window %.2f s left", r.gateTLeft)) : ""));
        put("turret err now / at release", f("%.2f / %.2f deg  (tol %.1f, rate %.0f deg/s, cap %.0f)", r.lastTurretErr, r.gateEstRelDeg,
                ShotControlTuning.gateTurRelTolDeg, shooterController.encoder.getVelocity(), ShotControlTuning.gateTurRateCapDps));
        put("wheel rpm / cmd", f("%.0f / %.0f  (tol %.0f)%s", shooterController.RPM, r.rpmCmd, ShotControlTuning.gateRpmTolRpm,
                shooterController.forceRpm > 0 ? "  SLIP FORCED" : ""));
        put("yaw rate / limit", f("%.0f / %.0f deg/s", odometry.getHeadingVelocityDeg(), ShotControlTuning.gateOmegaMaxDps));
        put("claim P / feasible / best ahead", f("%.3f / %s / %.3f", r.feasible ? r.pMake : 0.0, r.feasible, r.gateBestAhead));
        put("range (pivot->aim) / hood / turret cmd", f("%.1f in / %.3f deg / %.1f deg", r.result.d, r.hoodDeg, r.turretJavaDeg));
        put("controller", f("active %s targeting %s  %s", shooterController.reducedActive,
                shooterController.targeting, shooterController.reducedStatus));

        if (CAMPAIGN_ON) {
            put("CAMPAIGN", campaign.instruction());
            if (!campaign.done()) {
                put("at the stand?", campaign.atStand(r.result.d)
                        ? f("yes (axis range %.0f - 4 in wheel offset = %.0f; stand %d in)", r.result.d, r.result.d - ShotCampaign.WHEEL_FORWARD_IN, campaign.nominalRangeIn())
                        : f("NO: axis range %.0f - 4 = %.0f in wheel-to-goal, stand is %d in", r.result.d, r.result.d - ShotCampaign.WHEEL_FORWARD_IN, campaign.nominalRangeIn()));
            }
        }
        put("shots / last label", shotCount + " / " + ShotCsv.LABEL[lastLabel] + (pending != null ? "   (shot " + shotCount + " WAITING FOR A LABEL)" : ""));
        put("log", logStatus);
        put("status", status);
        put("battery", f("%.2f V", shooterController.batteryVolts));
    }
}
