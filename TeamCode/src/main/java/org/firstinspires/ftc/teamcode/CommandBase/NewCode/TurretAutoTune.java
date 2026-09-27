package org.firstinspires.ftc.teamcode.CommandBase.NewCode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.SetTurretAngle;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.ShooterController;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.ShooterController.ServoSelect;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.ShooterController.TurretDebugMode;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.TurretAutoTuner;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.TurretTune;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import java.io.File;
import java.io.FileWriter;
import java.io.Writer;
import java.util.Locale;

/*
 * Point the turret straight forward BEFORE pressing INIT (the encoder zeroes at init).
 * Robot on the ground and still, nothing in the way of the turret. Takes about 2-4 minutes.
 * The result is saved and loaded automatically by SetTurretAngle the next time an OpMode inits.
 * Every loop is logged to FIRST/settings/turret_tune_log.csv.
 */
@TeleOp(name = "Turret Auto Tune", group = "Tuning")
public class TurretAutoTune extends OpModeEX {

    private static final String LOG_FILE = "turret_tune_log.csv";
    private static final int LOG_MAX_LINES = 30000;

    private TurretAutoTuner tuner;
    private String saveStatus = "";
    private final StringBuilder log = new StringBuilder();
    private int logLines = 0;
    private boolean logWritten = false;
    private String logStatus = "";

    private final TurretAutoTuner.Io io = new TurretAutoTuner.Io() {
        @Override
        public double angle() {
            return shooterController.encoder.getTurretAngle();
        }

        @Override
        public double velocity() {
            return shooterController.encoder.getVelocity();
        }

        @Override
        public void setPower(double power) {
            shooterController.targeting = false;
            shooterController.manualServoSelect = ServoSelect.BOTH;
            shooterController.manualTurretPower = power;
            shooterController.turretDebugMode = TurretDebugMode.MANUAL;
        }

        @Override
        public void moveTo(double angle, TurretTune gains) {
            shooterController.targeting = false;
            gains.applyTo(shooterController.getTurretController());
            shooterController.testTurretTo(angle);
        }
    };

    @Override
    public void initEX() {
        telemetry.addLine("Turret must be pointing straight forward right now (encoder zeroed at INIT).");
        telemetry.addLine("Robot still, turret free to swing +/-" + (int) TurretTuning.safeRangeDeg + " deg.");
        telemetry.addLine("Press START to begin. Press STOP at any time to abort.");
        telemetry.update();
        log.append("t,stage,angle,velocity,mode,manualPower,target,profilePos,profileVel,output,servoCmd,status\n");
    }

    @Override
    public void loopEX() {
        if (tuner == null) {
            tuner = new TurretAutoTuner(io);
            tuner.safeRange = TurretTuning.safeRangeDeg;
            io.setPower(0);
        }

        tuner.step(getRuntime());
        logLoop();

        if (tuner.isDone() && saveStatus.isEmpty()) {
            TurretTune result = tuner.getResult();
            try {
                result.save();
                TurretTuning.copyFrom(result);
                saveStatus = "SAVED to " + TurretTune.defaultFile().getAbsolutePath();
            } catch (Exception e) {
                saveStatus = "SAVE FAILED: " + e.getMessage();
            }
        }
        if ((tuner.isDone() || tuner.isAborted()) && !logWritten) {
            writeLog();
        }

        telemetry.addData("stage", tuner.getStage());
        telemetry.addData("status", tuner.getMessage());
        if (!tuner.getWarning().isEmpty()) {
            telemetry.addData("warning", tuner.getWarning());
        }
        telemetry.addData("turret angle / vel", "%.1f deg / %.0f deg/s",
                shooterController.encoder.getTurretAngle(), shooterController.encoder.getVelocity());

        if (tuner.getDeadbandRamp() > 0) {
            telemetry.addData("deadband (breakaway power)", "%.3f", tuner.getDeadbandRamp());
        }
        if (tuner.getBaseTune() != null) {
            telemetry.addData("fit kS / kV", "%.4f / %.6f", tuner.getFitKS(), tuner.getFitKV());
            telemetry.addData("fit quality R2", "%.3f", tuner.getFitR2());
            telemetry.addData("kV + / - direction", "%.6f / %.6f", tuner.getKVPositive(), tuner.getKVNegative());
            telemetry.addData("top speed (deg/s)", "%.0f", tuner.getTopSpeed());
            telemetry.addData("delay / 63%% time (s)", "%.3f / %.3f", tuner.getMoveDelay(), tuner.getResponseTime());
        }
        if (!Double.isNaN(tuner.getGainLimitKP())) {
            telemetry.addData("kP stability limit", "%.5f%s", tuner.getGainLimitKP(),
                    tuner.isGainLimitFound() ? "" : " (no oscillation found)");
            telemetry.addData("candidates tried / failed", tuner.getCandidatesTried() + " / " + tuner.getCandidatesFailed());
        }
        if (tuner.getBestSoFar() != null) {
            telemetry.addData("best settle / overshoot", "%.2f s / %.2f deg", tuner.getBestSettle(), tuner.getBestOvershoot());
            telemetry.addData("best tracking error (RMS)", "%.2f deg", tuner.getBestTracking());
        }
        if (tuner.isDone()) {
            TurretTune r = tuner.getResult();
            telemetry.addLine("=== RESULT ===");
            telemetry.addData("kS", "%.4f", r.kS);
            telemetry.addData("kV", "%.6f", r.kV);
            telemetry.addData("kA", "%.6f", r.kA);
            telemetry.addData("kP", "%.5f", r.kP);
            telemetry.addData("kVelocityFeedback", "%.6f", r.kVelocityFeedback);
            telemetry.addData("maxCorrection", "%.2f", r.maxCorrection);
            telemetry.addData("maxVelocity", "%.0f", r.maxVelocity);
            telemetry.addData("maxAcceleration", "%.0f", r.maxAcceleration);
            telemetry.addData("latency", "%.3f", r.latency);
            telemetry.addData("speed/power table points", r.ffSpeed == null ? 0 : r.ffSpeed.length);
            telemetry.addLine(saveStatus);
        }
        if (!logStatus.isEmpty()) {
            telemetry.addLine(logStatus);
        }
    }

    private void logLoop() {
        if (logLines >= LOG_MAX_LINES) return;
        SetTurretAngle turret = shooterController.getTurretController();
        log.append(String.format(Locale.US, "%.4f,%s,%.3f,%.2f,%s,%.4f,%.3f,%.3f,%.2f,%.4f,%.4f,\"%s\"%n",
                getRuntime(),
                tuner.getStage(),
                shooterController.encoder.getTurretAngle(),
                shooterController.encoder.getVelocity(),
                shooterController.turretDebugMode,
                shooterController.manualTurretPower,
                turret.getTargetPosition(),
                turret.getProfilePosition(),
                turret.getProfileVelocity(),
                turret.getOutput(),
                turret.getServoCommand(),
                tuner.getMessage().replace("\"", "'")));
        logLines++;
    }

    private void writeLog() {
        logWritten = true;
        try {
            File file = AppUtil.getInstance().getSettingsFile(LOG_FILE);
            try (Writer out = new FileWriter(file)) {
                out.write(log.toString());
            }
            logStatus = "log saved: " + file.getAbsolutePath();
        } catch (Exception e) {
            logStatus = "LOG SAVE FAILED: " + e.getMessage();
        }
    }

    @Override
    public void stop() {
        ShooterController sc = shooterController;
        sc.manualTurretPower = 0;
        sc.turretDebugMode = TurretDebugMode.OFF;
        sc.getTurretController().stop();
        if (!logWritten) {
            writeLog();
        }
    }
}
