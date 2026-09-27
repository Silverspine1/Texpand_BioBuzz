package org.firstinspires.ftc.teamcode.CommandBase.NewCode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.ShooterController;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.ShooterController.ServoSelect;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.ShooterController.TurretDebugMode;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.TurretAutoTuner;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.TurretTune;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

/*
 * Point the turret straight forward BEFORE pressing INIT (the encoder zeroes at init).
 * Robot on the ground and still, nothing in the way of the turret. Takes about 2-4 minutes.
 * The result is saved and loaded automatically by SetTurretAngle the next time an OpMode inits.
 */
@TeleOp(name = "Turret Auto Tune", group = "Tuning")
public class TurretAutoTune extends OpModeEX {

    private TurretAutoTuner tuner;
    private String saveStatus = "";

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
    }

    @Override
    public void loopEX() {
        if (tuner == null) {
            tuner = new TurretAutoTuner(io);
            tuner.safeRange = TurretTuning.safeRangeDeg;
            io.setPower(0);
        }

        tuner.step(getRuntime());

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
            telemetry.addData("candidates tried", tuner.getCandidatesTried());
        }
        if (tuner.getResult() != null) {
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
            telemetry.addLine(saveStatus);
        }
    }

    @Override
    public void stop() {
        ShooterController sc = shooterController;
        sc.manualTurretPower = 0;
        sc.turretDebugMode = TurretDebugMode.OFF;
        sc.getTurretController().stop();
    }
}
