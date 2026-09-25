package org.firstinspires.ftc.teamcode.CommandBase.NewCode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Intake.intakeHardware;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.AxonEncoder;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.SetTurretAngle;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.ShooterController.ServoSelect;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.ShooterController.TurretDebugMode;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.Shotplanner;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;
@TeleOp
public class Test_tele extends OpModeEX {

    private static final double CHECK_WINDOW_S = 0.25;

    private final ElapsedTime turretWindow = new ElapsedTime();
    private double windowStartAngle = 0;
    private double windowStartVoltage = 0;
    private double measuredTurretVel = 0;
    private String turretDirectionVerdict = "move the turret (MANUAL, stick right)";
    private String encoderVelVerdict = "-";
    private String voltageVerdict = "-";

    private final ElapsedTime headingWindow = new ElapsedTime();
    private double windowStartHeading = 0;
    private String headingVerdict = "rotate the robot left (CCW)";

    @Override
    public void initEX() {

    }

    @Override
    public void loopEX() {
        driveBase.drivePowers(gamepad1.right_stick_y, (gamepad1.left_trigger- gamepad1.right_trigger),gamepad1.left_stick_x);
        if (gamepad1.right_bumper){
            IntakeHardware.State = intakeHardware.intakeState.intaking;
        }else if(!transfer.transfer) {
            IntakeHardware.State = intakeHardware.intakeState.off;
        }
        if (!lastGamepad1.left_bumper && currentGamepad1.left_bumper){
            transfer.Trans(true,500);
        }
        if (!lastGamepad1.dpad_up && currentGamepad1.dpad_up && !shooterController.targeting){
            shooterController.targeting = true;
        } else if (!lastGamepad1.dpad_up && currentGamepad1.dpad_up && shooterController.targeting) {
            shooterController.targeting = false;
        }

        turretDebugControls();

        AxonEncoder encoder = shooterController.encoder;
        SetTurretAngle turret = shooterController.getTurretController();

        checkTurretDirection(encoder);
        checkHeadingDirection();

        telemetry.addLine("=== ODOMETRY ===");
        telemetry.addData("X / Y (cm)", "%.1f / %.1f", odometry.X(), odometry.Y());
        telemetry.addData("H (deg)", "%.1f", odometry.Heading());
        telemetry.addData("XVel / YVel (cm/s)", "%.1f / %.1f", odometry.getXVelocity(), odometry.getYVelocity());
        telemetry.addData("HVel (rad/s!)", "%.3f  = %.1f deg/s", odometry.getHeadingVelocity(), Math.toDegrees(odometry.getHeadingVelocity()));
        telemetry.addData("Heading dir check", headingVerdict);

        telemetry.addLine("=== TURRET ENCODER ===");
        telemetry.addData("voltage (V)", "%.3f", encoder.getVoltage());
        telemetry.addData("raw servo deg", "%.1f", encoder.getRawDegrees());
        telemetry.addData("total servo deg", "%.1f", encoder.getTotalPosition());
        telemetry.addData("turret angle (deg)", "%.2f", encoder.getTurretAngle());
        telemetry.addData("encoder.getVelocity()", flagNaN(encoder.getVelocity()));
        telemetry.addData("measured turret vel (deg/s)", "%.1f", measuredTurretVel);
        telemetry.addData("voltage vs angle", voltageVerdict);
        telemetry.addData("encoder vel sign", encoderVelVerdict);

        telemetry.addLine("=== TURRET SERVOS ===");
        telemetry.addData("debug mode", shooterController.turretDebugMode);
        telemetry.addData("servo select", shooterController.manualServoSelect);
        telemetry.addData("manual power", "%.2f  (servo cmd %.2f)", shooterController.manualTurretPower, 0.5 + shooterController.manualTurretPower);
        telemetry.addData("DIRECTION", turretDirectionVerdict);
        telemetry.addData("ctrl active / atTarget", turret.isActive() + " / " + turret.atTarget());
        telemetry.addData("ctrl target (deg)", "%.1f", turret.getTargetPosition());
        telemetry.addData("ctrl profile pos/vel", "%.1f / %.1f", turret.getProfilePosition(), turret.getProfileVelocity());
        telemetry.addData("ctrl error (deg)", "%.2f", turret.getPositionError());
        telemetry.addData("ctrl output", flagNaN(turret.getOutput()));
        telemetry.addData("ctrl servo cmd", flagNaN(turret.getServoCommand()));

        telemetry.addLine("=== SHOT PLANNER ===");
        telemetry.addData("targeting", shooterController.targeting);
        telemetry.addData("flywheel RPM", "%.0f", shooterController.RPM);
        Shotplanner.ShotSolution shot = shooterController.lastShot;
        if (shot != null) {
            telemetry.addData("reachable / safe", shot.reachable + " / " + shot.safe);
            telemetry.addData("turret angle cmd", flagNaN(shot.turretAngleDeg));
            telemetry.addData("launch angle", flagNaN(shot.launchAngleDeg));
            telemetry.addData("target RPM", flagNaN(shot.flywheelRPM));
            telemetry.addData("clearance (m)", "%.3f", shot.minimumClearanceMeters);
            telemetry.addData("planner time (ms)", "%.1f", shooterController.plannerMs);

            Shotplanner.HiveTarget hive = Shotplanner.getBlueHiveTarget(Shotplanner.BlueHiveSide.AUDIENCE);
            double bearing = Math.toDegrees(Math.atan2(hive.openingY - odometry.Y() / 100, hive.openingX - odometry.X() / 100));
            telemetry.addData("geometric bearing (deg)", "%.1f", normalize(bearing - odometry.Heading()));
        }
        Shotplanner.RobotState rs = shooterController.lastRobotState;
        if (rs != null) {
            telemetry.addData("planner in: x,y (m)", "%.2f, %.2f", rs.x, rs.y);
            telemetry.addData("planner in: vx,vy (m/s)", "%.2f, %.2f", rs.velocityX, rs.velocityY);
            telemetry.addData("planner in: ax,ay (m/s2)", "%.2f, %.2f", rs.accelerationX, rs.accelerationY);
            telemetry.addData("planner in: heading / omega", "%.1f deg / %.3f deg/s", rs.headingDeg, rs.angularVelocityDegPerSec);
            telemetry.addData("planner in: turret omega", flagNaN(rs.turretAngularVelocityDegPerSec));
        }
    }

    private void turretDebugControls() {
        if (!lastGamepad1.y && currentGamepad1.y) {
            shooterController.turretDebugMode =
                    shooterController.turretDebugMode == TurretDebugMode.OFF
                            ? TurretDebugMode.MANUAL
                            : TurretDebugMode.OFF;
        }

        if (!lastGamepad1.x && currentGamepad1.x) {
            ServoSelect[] options = ServoSelect.values();
            shooterController.manualServoSelect =
                    options[(shooterController.manualServoSelect.ordinal() + 1) % options.length];
        }

        if (!lastGamepad1.b && currentGamepad1.b) {
            shooterController.encoder.resetTurretAngle();
        }

        if (!lastGamepad1.dpad_left && currentGamepad1.dpad_left) {
            shooterController.testTurretTo(45);
        }
        if (!lastGamepad1.dpad_right && currentGamepad1.dpad_right) {
            shooterController.testTurretTo(-45);
        }
        if (!lastGamepad1.dpad_down && currentGamepad1.dpad_down) {
            shooterController.testTurretTo(0);
        }

        double stick = gamepad1.right_stick_x;
        shooterController.manualTurretPower = Math.abs(stick) > 0.05 ? stick * 0.3 : 0;
    }

    private void checkTurretDirection(AxonEncoder encoder) {
        if (turretWindow.seconds() < CHECK_WINDOW_S) {
            return;
        }

        double angle = encoder.getTurretAngle();
        double voltage = encoder.getVoltage();
        double dAngle = angle - windowStartAngle;
        double dVoltage = voltage - windowStartVoltage;
        measuredTurretVel = dAngle / turretWindow.seconds();

        if (Math.abs(dAngle) > 2.0) {
            if (Math.abs(dVoltage) < 1.0) {
                voltageVerdict = (Math.signum(dVoltage) == -Math.signum(dAngle))
                        ? "voltage DOWN when angle UP (as code expects)"
                        : "voltage UP when angle UP";
            }

            double reported = encoder.getVelocity();
            if (Double.isNaN(reported) || Double.isInfinite(reported)) {
                encoderVelVerdict = "BROKEN: " + reported;
            } else {
                encoderVelVerdict = Math.signum(reported) == Math.signum(dAngle) ? "OK (matches angle change)" : "WRONG SIGN";
            }

            double power = shooterController.manualTurretPower;
            if (shooterController.turretDebugMode == TurretDebugMode.MANUAL &&Math.abs(power) > 0.05) {
                turretDirectionVerdict = Math.signum(power) == Math.signum(dAngle)
                        ? "OK: + power -> angle UP (" + shooterController.manualServoSelect + ")"
                        : "REVERSED: + power -> angle DOWN (" + shooterController.manualServoSelect + ")";
            }
        }

        windowStartAngle = angle;
        windowStartVoltage = voltage;
        turretWindow.reset();
    }

    private void checkHeadingDirection() {
        if (headingWindow.seconds() < CHECK_WINDOW_S) {
            return;
        }

        double heading = odometry.Heading();
        double dHeading = normalize(heading - windowStartHeading);

        if (Math.abs(dHeading) > 3.0) {
            double hVel = odometry.getHeadingVelocity();
            headingVerdict = (dHeading > 0 ? "heading UP" : "heading DOWN")
                    + (Math.signum(hVel) == Math.signum(dHeading)
                    ? ", HVel sign MATCHES"
                    : ", HVel sign OPPOSITE (negate it for planner)");
        }

        windowStartHeading = heading;
        headingWindow.reset();
    }

    private static String flagNaN(double value) {
        if (Double.isNaN(value) || Double.isInfinite(value)) {
            return "!!! " + value + " !!!";
        }
        return String.format("%.3f", value);
    }

    private static double normalize(double angle) {
        angle %= 360.0;
        if (angle > 180) angle -= 360;
        if (angle <= -180) angle += 360;
        return angle;
    }
}
