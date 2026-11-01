package org.firstinspires.ftc.teamcode.CommandBase.NewCode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathingUtility.RobotPower;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Intake.intakeHardware_doubleBot;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.AxonEncoder;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.ReducedShotController;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.SetTurretAngle;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.ShotHold;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.ShooterController;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.ShooterController.ServoSelect;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.ShooterController.TurretDebugMode;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.TurretTune;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;
@TeleOp
public class Test_tele extends OpModeEX {

    private static final double CHECK_WINDOW_S = 0.25;


    private final ElapsedTime turretWindow = new ElapsedTime();
    private double windowStartAngle = 0;
    private double windowStartVoltage = 0;
    private String turretDirectionVerdict = "move the turret (MANUAL, stick right)";
    private String encoderVelVerdict = "spin or move the turret";
    private final ElapsedTime velClock = new ElapsedTime();
    private final ElapsedTime velWindow = new ElapsedTime();
    private double lastVelAngle = Double.NaN;
    private int velAgree = 0;
    private int velTotal = 0;

    private final ShotHold shotHold = new ShotHold();
    private final ElapsedTime packetTimer = new ElapsedTime();
    private String voltageVerdict = "-";

    private boolean pingPong = false;
    private boolean pingPongPositive = true;
    private final ElapsedTime pingPongTimer = new ElapsedTime();

    private final ElapsedTime moveTimer = new ElapsedTime();
    private final ElapsedTime Loop = new ElapsedTime();

    private boolean moveInProgress = false;
    private double moveStartAngle = 0;
    private double moveTarget = 0;
    private double moveOvershoot = 0;
    private double lastSettleTime = 0;
    private double lastOvershoot = 0;
    private double kVEstimate = 0;
    double loop = 0;

    private String tuneStatus = "defaults (no saved tune)";
    private String poseStatus = "";

    private final ElapsedTime headingWindow = new ElapsedTime();
    private double windowStartHeading = 0;
    private String headingVerdict = "rotate the robot left (CCW)";

    @Override
    public void initEX() {
        TurretTune saved = TurretTune.load();
        if (saved != null) {
            TurretTuning.copyFrom(saved);
            tuneStatus = "loaded saved tune";
        }
        odometry.setPose(ShooterTuning.startXcm, ShooterTuning.startYcm, ShooterTuning.startHeadingDeg);
        vision.enable();   // opens the camera for the vision collect button (gamepad1 START)
    }

    @Override
    public void loopEX() {
        Loop.reset();

        // ---- VISION COLLECT: gamepad1 START toggles it. It drives the chassis along the fastest path that sweeps the
        // balls into the intake and holds the intake on; it never touches the turret / flywheel / shooting.
        // It ignores where it ends up (VisionSubsystem.CONSIDER_END_POSITION is off). Any driver input cancels it.
        if (!lastGamepad1.start && currentGamepad1.start) {
            if (vision.isCollecting()) vision.stopCollect(); else vision.startCollect();
        }
        if (vision.isCollecting() && (Math.abs(gamepad1.right_stick_y) > VisionSubsystem.DRIVER_OVERRIDE_STICK
                || Math.abs(gamepad1.right_stick_x) > VisionSubsystem.DRIVER_OVERRIDE_STICK
                || gamepad1.left_trigger > 0.2 || gamepad1.right_trigger > 0.2
                || gamepad1.right_bumper || gamepad1.left_bumper)) {
            vision.stopCollect();
        }

        if (vision.isCollecting()) {
            RobotPower visionPower = vision.getDrivePower();
            if (visionPower != null) driveBase.drivePowers(visionPower);
            else driveBase.drivePowers(0, 0, 0);
            IntakeHardware.Vertical = -1;
            IntakeHardware.State = intakeHardware_doubleBot.intakeState.intaking;
        } else {
        driveBase.drivePowers(gamepad1.right_stick_y,-gamepad1.right_stick_x,(gamepad1.left_trigger- gamepad1.right_trigger));
        IntakeHardware.Vertical = driveBase.vertical;
        // FEED = gamepad1 left bumper. With the reduced controller active the fire gate HOLDS it until FireGate says FIRE
        // (ShotControlTuning.gateHoldsFire; gamepad2 B = override, gamepad2 left bumper = movement protection ON/OFF).
        // Not applicable (controller off, no model, hold disabled) = the old behaviour: the bumper feeds directly.
        boolean feedApplicable = ShotControlTuning.gateHoldsFire && shooterController.reducedActive
                && shooterController.reduced.hasModel();
        boolean feedNow = shotHold.update(shooterController.reducedSeconds(), gamepad1.left_bumper, gamepad2.b,
                feedApplicable, shooterController.reduced.gateState);
        if (shotHold.consumeRequestOpened()) shooterController.requestReducedShot();
        if (shotHold.feedStarted) shooterController.notifyReducedShot();
        if (gamepad1.right_bumper){
            IntakeHardware.State = intakeHardware_doubleBot.intakeState.intaking;
        }else if(!feedNow) {
            IntakeHardware.State = intakeHardware_doubleBot.intakeState.off;
        }
        if (feedNow){
            IntakeHardware.State = intakeHardware_doubleBot.intakeState.transfering;
        }else if (!gamepad1.right_bumper){
            IntakeHardware.State = intakeHardware_doubleBot.intakeState.off;


        }
        }        if (!lastGamepad1.dpad_up && currentGamepad1.dpad_up) {
            shooterController.targeting = !shooterController.targeting;
            if (shooterController.targeting) {
                pingPong = false;
                shooterController.turretDebugMode = TurretDebugMode.OFF;
            }
        }

        AxonEncoder encoder = shooterController.encoder;
        SetTurretAngle turret = shooterController.getTurretController();

        turret.setGains(
                TurretTuning.kS,
                TurretTuning.kV,
                TurretTuning.kA,
                TurretTuning.kP,
                TurretTuning.kVelocityFeedback,
                TurretTuning.maxCorrection
        );
        turret.setConstraints(TurretTuning.maxVelocity, TurretTuning.maxAcceleration);
        turret.setLatencyCompensation(TurretTuning.latency);
        turret.setPlant(TurretTuning.plantDelay, TurretTuning.plantAccel, TurretTuning.plantDecel, TurretTuning.kPos, TurretTuning.decelSafety);
        turret.setBrakeAware(TurretTuning.brakeAware, TurretTuning.brakeDelayScale);

        turretDebugControls();
        trackMove(encoder, turret);
        sendDashboardGraph(encoder, turret);

        checkTurretDirection(encoder);
        checkVelocityAgreement(encoder);
        checkHeadingDirection();

        telemetry.addLine("=== ODOMETRY ===");
        telemetry.addData("X / Y (cm)", "%.1f / %.1f", odometry.X(), odometry.Y());
        telemetry.addData("H (deg)", "%.1f", odometry.Heading());
        telemetry.addData("XVel / YVel (cm/s)", "%.1f / %.1f", odometry.getXVelocity(), odometry.getYVelocity());
        telemetry.addData("heading vel (deg/s)", "%.1f", odometry.getHeadingVelocityDeg());
        telemetry.addData("Heading dir check", headingVerdict);
        telemetry.addData("pose (R-stick btn sets)", "%.0f, %.0f cm @ %.0f deg  %s",
                ShooterTuning.startXcm, ShooterTuning.startYcm, ShooterTuning.startHeadingDeg, poseStatus);

        ShotDebug.pose(odometry, shooterController, telemetry::addData);
        vision.addTelemetry(telemetry);

        telemetry.addLine("=== TURRET ENCODER ===");
        telemetry.addData("voltage (V)", "%.3f", encoder.getVoltage());
        telemetry.addData("raw servo deg", "%.1f", encoder.getRawDegrees());
        telemetry.addData("total servo deg", "%.1f", encoder.getTotalPosition());
        telemetry.addData("turret angle (deg)", "%.2f", encoder.getTurretAngle());
        telemetry.addData("turret velocity (deg/s)", flagNaN(encoder.getVelocity()));
        telemetry.addData("velocity vs angle change", encoderVelVerdict);
        telemetry.addData("voltage vs angle", voltageVerdict);

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
        telemetry.addData("tune (L-stick btn saves both)", tuneStatus);
        telemetry.addData("ping-pong (A)", pingPong);
        telemetry.addData("last move settle (s)", "%.2f", lastSettleTime);
        telemetry.addData("last move overshoot (deg)", "%.2f", lastOvershoot);
        telemetry.addData("kV estimate (manual)", "%.5f", kVEstimate);

        telemetry.addLine("=== FLYWHEEL ===");
        shooterController.manualFlywheelPower = gamepad2.right_trigger;
        telemetry.addData("targeting (dpad up)", shooterController.targeting);
        telemetry.addData("manual flywheel (gamepad2 R-trigger)", "%.2f  raw %.0f ticks/s", shooterController.manualFlywheelPower,
                shooterController.flywheelTicksPerSecond);
        telemetry.addData("flywheel RPM / target", "%.0f / %.0f  power %.2f  %s", shooterController.RPM,
                shooterController.getTargetRPM(), shooterController.flywheelPower,
                shooterController.flywheelReady() ? "READY" : "");
        if (shooterController.flywheelReady() && shooterController.flywheelPower > 0.05) {
            telemetry.addData("kF if steady", "%.6f", shooterController.flywheelPower / shooterController.getTargetRPM());
        }
        reducedShotControls();
        reducedShotTelemetry();
        ShotDebug.flywheel(shooterController, telemetry::addData);
        loop = Loop.milliseconds();
        telemetry.addData("loot time(ms)", "%.1f", loop);

    }

    /**
     * Under-shooter REDUCED shot control (ShotControlTuning, always on), gamepad2:
     * dpad up/down = exit-speed trim +-, dpad left/right = turret trim -/+, BACK clears the trims,
     * X reloads the model files, START saves ShotControlTuning (not 'enabled': it always starts OFF),
     * A opens a fire-gate request window (gateWindowS), LEFT BUMPER toggles MOVEMENT PROTECTION (settle gate) ON/OFF
     * (default ON; always starts ON), B (hold) = feed override. gamepad1 LEFT BUMPER feeds: held until the gate says FIRE.
     */
    private void reducedShotControls() {
        String trim = ShotControlTuning.trimButtons(currentGamepad2, lastGamepad2);
        if (trim != null) reducedSaveStatus = trim + " (START saves)";
        if (!lastGamepad2.x && currentGamepad2.x) shooterController.loadReducedModel();
        if (!lastGamepad2.a && currentGamepad2.a) shooterController.requestReducedShot();
        if (!lastGamepad2.left_bumper && currentGamepad2.left_bumper) ShotControlTuning.gateProtect = !ShotControlTuning.gateProtect;
        if (!lastGamepad2.start && currentGamepad2.start) {
            try {
                ShotControlTuning.save();
                reducedSaveStatus = "saved";
            } catch (Exception e) {
                reducedSaveStatus = "save failed: " + e.getMessage();
            }
        }
    }

    private String reducedSaveStatus = "";

    private void reducedShotTelemetry() {
        ReducedShotController r = shooterController.reduced;
        telemetry.addLine("=== REDUCED SHOT CONTROL ===");
        telemetry.addData("active", "%s  (%s alliance)", shooterController.reducedActive, ShotControlTuning.blueAlliance ? "BLUE" : "RED");
        telemetry.addData("model", shooterController.reducedStatus);
        if (!r.hasModel()) return;
        telemetry.addData("aim: team x,y,h", "%.1f, %.1f in, %.1f deg  d %.1f in", r.teamX, r.teamY, r.teamHeading, r.result.d);
        telemetry.addData("aim: vr / vt (in/s)", "%.1f / %.1f", r.result.vr, r.result.vt);
        telemetry.addData("rpm cmd / at release", "%.0f / %.0f  (wheel %.0f)  max %.0f @ %.2f V", r.rpmCmd,
                r.rpmAtReleaseReal, shooterController.RPM, r.rpmMaxEff, shooterController.batteryVolts);
        telemetry.addData("hood cmd (deg travel)", "%.3f  servo pos %.4f (0.182 bottom .. 0.218 top)", r.hoodDeg,
                ShooterController.underHoodServoPos(r.hoodDeg));
        telemetry.addData("turret cmd / err (deg)", "%.2f / %.2f  (lead %.2f)", r.turretJavaDeg, r.lastTurretErr, r.result.turretLeadDeg);
        telemetry.addData("claimed P(make)", "%.3f  feasible %s  occ %.2f  tof %.2f s", r.pMake, r.feasible, r.result.occF, r.result.tof);
        telemetry.addData("FIRE GATE", "%s%s", ReducedShotController.GATE_TEXT[r.gateState],
                r.gateState == ReducedShotController.GATE_UNSETTLED ? " (" + ReducedShotController.REFUSE_TEXT[r.gateRefuse] + ")" : "");
        telemetry.addData("gate: P now / best ahead", "%.3f / %.3f  t_left %.2f s%s  cmd rates %.0f deg/s %.0f rpm/s",
                r.feasible ? r.pMake : 0.0, r.gateBestAhead, r.gateTLeft,
                r.requestActive ? (r.gateDeadline ? " DEADLINE" : " (request)") : "", r.turCmdRate, r.rpmCmdRate);
        telemetry.addData("MOVEMENT PROTECTION", ShotControlTuning.gateProtect ? "ON (gamepad2 L-bumper = off)" : "OFF !!! (gamepad2 L-bumper = on)");
        telemetry.addData("feed hold", "%s  predicted turret err at release %.2f deg", ShotHold.TEXT[shotHold.state], r.gateEstRelDeg);
        telemetry.addData("burst: index / detected", "%d / %d", r.shotIndex, r.shotsDetected);
        telemetry.addData("solve / look-ahead / gate (us)", "%.1f / %.1f / %.1f  max %.1f", r.solveNanos / 1e3,
                r.lookNanos / 1e3, r.gateNanos / 1e3, r.maxSolveNanos / 1e3);
        telemetry.addData("TUNE", "%s  %s", ShotControlTuning.tuneLine(), reducedSaveStatus);
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


        if (!lastGamepad1.left_stick_button && currentGamepad1.left_stick_button) {
            TurretTune tune = TurretTuning.toTune();
            if (!tune.isValid()) {
                tuneStatus = "NOT SAVED: values invalid";
            } else {
                try {
                    tune.save();
                    ShooterTuning.save();
                    tuneStatus = "saved turret + shooter";
                } catch (Exception e) {
                    tuneStatus = "SAVE FAILED: " + e.getMessage();
                }
            }
        }

        if (!lastGamepad1.right_stick_button && currentGamepad1.right_stick_button) {
            odometry.setPose(ShooterTuning.startXcm, ShooterTuning.startYcm, ShooterTuning.startHeadingDeg);
            poseStatus = "set";
        }

        if (!lastGamepad1.b && currentGamepad1.b) {
            shooterController.encoder.resetTurretAngle();
        }

        if (!lastGamepad1.dpad_left && currentGamepad1.dpad_left) {
            pingPong = false;
            startMove(TurretTuning.testAngle);
        }
        if (!lastGamepad1.dpad_right && currentGamepad1.dpad_right) {
            pingPong = false;
            startMove(-TurretTuning.testAngle);
        }
        if (!lastGamepad1.dpad_down && currentGamepad1.dpad_down) {
            pingPong = false;
            startMove(0);
        }

        if (!lastGamepad1.a && currentGamepad1.a) {
            pingPong = !pingPong;
            pingPongTimer.reset();
            if (pingPong) {
                pingPongPositive = true;
                startMove(TurretTuning.testAngle);
            }
        }
        if (pingPong && shooterController.turretDebugMode == TurretDebugMode.CLOSED_LOOP
                && pingPongTimer.seconds() >= TurretTuning.pingPongSeconds) {
            pingPongTimer.reset();
            pingPongPositive = !pingPongPositive;
            startMove(pingPongPositive ? TurretTuning.testAngle : -TurretTuning.testAngle);
        }

        double stick = gamepad1.right_stick_x;
        shooterController.manualTurretPower = Math.abs(stick) > 0.05 ? stick * TurretTuning.manualMaxPower : 0;
    }

    private void startMove(double angle) {
        moveStartAngle = shooterController.encoder.getTurretAngle();
        moveTarget = angle;
        moveOvershoot = 0;
        moveInProgress = true;
        moveTimer.reset();
        shooterController.testTurretTo(angle);
    }

    private void trackMove(AxonEncoder encoder, SetTurretAngle turret) {
        if (!moveInProgress) {
            return;
        }

        double direction = Math.signum(moveTarget - moveStartAngle);
        double past = (encoder.getTurretAngle() - moveTarget) * direction;
        moveOvershoot = Math.max(moveOvershoot, past);

        if (turret.atTarget()) {
            lastSettleTime = moveTimer.seconds();
            lastOvershoot = moveOvershoot;
            moveInProgress = false;
        }
    }

    private void sendDashboardGraph(AxonEncoder encoder, SetTurretAngle turret) {
        if (packetTimer.milliseconds() < 33) {
            return;
        }
        packetTimer.reset();


    }

    private void checkTurretDirection(AxonEncoder encoder) {
        if (turretWindow.seconds() < CHECK_WINDOW_S) {
            return;
        }

        double angle = encoder.getTurretAngle();
        double voltage = encoder.getVoltage();
        double dAngle = angle - windowStartAngle;
        double dVoltage = voltage - windowStartVoltage;
        if (Math.abs(dAngle) > 2.0) {
            if (Math.abs(dVoltage) < 1.0) {
                voltageVerdict = (Math.signum(dVoltage) == -Math.signum(dAngle))
                        ? "voltage DOWN when angle UP (as code expects)"
                        : "voltage UP when angle UP";
            }

            double power = shooterController.manualTurretPower;
            if (shooterController.turretDebugMode == TurretDebugMode.MANUAL && Math.abs(power) > 0.05) {
                if (Math.abs(encoder.getVelocity()) > 20) {
                    kVEstimate = power / encoder.getVelocity();
                }
                turretDirectionVerdict = Math.signum(power) == Math.signum(dAngle)
                        ? "OK: + power -> angle UP (" + shooterController.manualServoSelect + ")"
                        : "REVERSED: + power -> angle DOWN (" + shooterController.manualServoSelect + ")";
            }
        }

        windowStartAngle = angle;
        windowStartVoltage = voltage;
        turretWindow.reset();
    }

    /** Compares AxonEncoder.getVelocity() with the change in AxonEncoder.getTurretAngle() every loop, over one second. */
    private void checkVelocityAgreement(AxonEncoder encoder) {
        double angle = encoder.getTurretAngle();
        double dt = velClock.seconds();
        velClock.reset();

        double reported = encoder.getVelocity();
        if (Double.isNaN(reported) || Double.isInfinite(reported)) {
            encoderVelVerdict = "BROKEN: " + reported;
            return;
        }

        if (!Double.isNaN(lastVelAngle) && dt > 0) {
            double derived = (angle - lastVelAngle) / dt;
            if (Math.abs(derived) > 60 && Math.abs(reported) > 60) {
                velTotal++;
                if (Math.signum(derived) == Math.signum(reported)) velAgree++;
            }
        }
        lastVelAngle = angle;

        if (velWindow.seconds() >= 1.0) {
            if (velTotal >= 5) {
                double percent = 100.0 * velAgree / velTotal;
                encoderVelVerdict = percent >= 85 ? String.format("OK, matches angle change (%.0f%%)", percent)
                        : percent <= 15 ? String.format("REVERSED (%.0f%%): flip AxonEncoder.TURRET_DIRECTION", percent)
                        : String.format("mixed (%.0f%%)", percent);
            }
            velAgree = 0;
            velTotal = 0;
            velWindow.reset();
        }
    }

    private void checkHeadingDirection() {
        if (headingWindow.seconds() < CHECK_WINDOW_S) {
            return;
        }

        double heading = odometry.Heading();
        double dHeading = normalize(heading - windowStartHeading);

        if (Math.abs(dHeading) > 3.0) {
            double hVel = odometry.getHeadingVelocityDeg();
            headingVerdict = (dHeading > 0 ? "heading UP" : "heading DOWN")
                    + (Math.signum(hVel) == Math.signum(dHeading)
                    ? ", velocity sign MATCHES"
                    : ", velocity sign OPPOSITE (fix Odometry.getHeadingVelocityDeg)");
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
