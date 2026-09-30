package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.ShooterTuning;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Localisation.Odometry;
import org.firstinspires.ftc.teamcode.CommandBase.LoopProfiler;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_command_base.Hardware.MotorEx;
import dev.weaponboy.nexus_command_base.Hardware.ServoDegrees;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;


public class ShooterController extends SubSystem {

    Shotplanner shotPlanner = new Shotplanner(new Shotplanner.Config());
    ElapsedTime plannerTimer = new ElapsedTime();
    ElapsedTime loopTimer = new ElapsedTime();
    FlywheelController flywheel = new FlywheelController();
    PlannerInputFilter inputFilter = new PlannerInputFilter();
    ShotSolutionHold solutionHold = new ShotSolutionHold();
    boolean wasTargeting = false;
    public AxonEncoder encoder;
    Odometry OdoMetry;

    public Shotplanner.BlueHiveSide blueHiveSide = Shotplanner.BlueHiveSide.AUDIENCE;
    ServoDegrees Hood = new ServoDegrees();
    Servo turretServo1;
    Servo turretServo2;
    SetTurretAngle turretController;

    public MotorEx shooterMotor = new MotorEx();
    double targetRPM = 0;
    public double RPM;
    public double flywheelPower = 0;
    public boolean targeting = false;

    public enum TurretDebugMode { OFF, MANUAL, CLOSED_LOOP }
    public enum ServoSelect { BOTH, SERVO1_ONLY, SERVO2_ONLY }

    public TurretDebugMode turretDebugMode = TurretDebugMode.OFF;
    private TurretDebugMode lastTurretDebugMode = TurretDebugMode.OFF;
    public ServoSelect manualServoSelect = ServoSelect.BOTH;
    public double manualTurretPower = 0;

    public Shotplanner.ShotSolution lastShot;
    public Shotplanner.RobotState lastRobotState;
    public double plannerMs = 0;

    public SetTurretAngle getTurretController() {
        return turretController;
    }

    public void testTurretTo(double angleDeg) {
        targeting = false;
        turretDebugMode = TurretDebugMode.CLOSED_LOOP;
        turretController.setTarget(angleDeg, 1.0, false);
    }

    public ShooterController(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, returnDefaultCommand());
        this.OdoMetry = opModeEX.odometry;
    }

    @Override
    public void init() {
        ShooterTuning.load();

        shooterMotor.initMotor("shooterMotor", getOpMode().hardwareMap);

        Hood.initServo("hoodServ",getOpMode().hardwareMap);
        turretServo1 = getOpMode().hardwareMap.get(Servo.class, "turretServo1");
        turretServo2 = getOpMode().hardwareMap.get(Servo.class, "turretServo2");

        encoder = new AxonEncoder();
        encoder.init(getOpMode().hardwareMap, "turretEncoder");
        Hood.setRange(355);

        turretController = new SetTurretAngle(
                turretServo1,
                turretServo2,
                encoder,
                220.0,    // max turret velocity, deg/s
                1200.0    // max turret acceleration, deg/s²
        );

        applyTuning();
        loopTimer.reset();
    }

    private void applyTuning() {
        Shotplanner.Config c = shotPlanner.getConfig();
        c.flywheelRadiusMeters = Math.max(0.001, ShooterTuning.flywheelRadiusMeters);
        c.muzzleEfficiency = Math.max(0.05, ShooterTuning.muzzleEfficiency);
        c.magnusK = ShooterTuning.magnusK;
        c.maxFlywheelRPM = Math.max(100, ShooterTuning.maxFlywheelRPM);
        c.requiredClearanceMeters = ShooterTuning.requiredClearanceMeters;
        c.muzzleHeightMeters = ShooterTuning.muzzleHeightMeters;
        c.turretAxisForwardMeters = ShooterTuning.turretAxisForwardMeters;
        c.muzzleForwardFromTurretMeters = ShooterTuning.muzzleForwardFromTurretMeters;

        double low = Math.min(ShooterTuning.hoodLaunchAtMinDeg, ShooterTuning.hoodLaunchAtMaxDeg);
        double high = Math.max(ShooterTuning.hoodLaunchAtMinDeg, ShooterTuning.hoodLaunchAtMaxDeg);
        c.minLaunchAngleDeg = low;
        c.maxLaunchAngleDeg = Math.max(high, low + 1);

        flywheel.kF = ShooterTuning.flywheelKF;
        flywheel.kP = ShooterTuning.flywheelKP;
        flywheel.kI = ShooterTuning.flywheelKI;
        flywheel.integralMax = ShooterTuning.flywheelIntegralMax;
        flywheel.integralZoneRpm = ShooterTuning.flywheelIntegralZoneRpm;
        flywheel.maxPower = Math.max(0, Math.min(1, ShooterTuning.flywheelMaxPower));

        inputFilter.timeConstantSeconds = ShooterTuning.plannerVelocityFilterSeconds;
        inputFilter.velocityDeadbandMps = ShooterTuning.plannerVelocityDeadbandMps;
        inputFilter.headingRateDeadbandDps = ShooterTuning.plannerHeadingRateDeadbandDps;
        inputFilter.turretRateDeadbandDps = ShooterTuning.plannerTurretRateDeadbandDps;
        inputFilter.useAcceleration = ShooterTuning.plannerUseAcceleration;
        solutionHold.holdSolves = (int) Math.max(0, ShooterTuning.plannerHoldSolves);
    }

    public void setHoodDegrees(double theta) {
        double launchSpan = ShooterTuning.hoodLaunchAtMaxDeg - ShooterTuning.hoodLaunchAtMinDeg;
        double servoSpan = ShooterTuning.hoodServoAtMaxDeg - ShooterTuning.hoodServoAtMinDeg;
        double fraction = Math.abs(launchSpan) < 1e-6 ? 0 : (theta - ShooterTuning.hoodLaunchAtMinDeg) / launchSpan;
        double servoPos = ShooterTuning.hoodServoAtMinDeg + fraction * servoSpan;

        double lowLimit = Math.min(ShooterTuning.hoodServoAtMinDeg, ShooterTuning.hoodServoAtMaxDeg);
        double highLimit = Math.max(ShooterTuning.hoodServoAtMinDeg, ShooterTuning.hoodServoAtMaxDeg);
        Hood.setPosition(Math.max(lowLimit, Math.min(highLimit, servoPos)));
    }

    public Shotplanner.HiveTarget getHiveTarget() {
        return getHiveTarget(blueHiveSide);
    }

    public static Shotplanner.HiveTarget getHiveTarget(Shotplanner.BlueHiveSide side) {
        boolean audience = side == Shotplanner.BlueHiveSide.AUDIENCE;
        return new Shotplanner.HiveTarget(
                audience ? ShooterTuning.audienceGoalX : ShooterTuning.oppositeGoalX,
                audience ? ShooterTuning.audienceGoalY : ShooterTuning.oppositeGoalY,
                ShooterTuning.openingBottomZ,
                audience ? 180.0 : 0.0
        );
    }

    public Shotplanner.Config getPlannerConfig() {
        return shotPlanner.getConfig();
    }

    /** Turret angle (robot-relative, CCW +) that points straight at the goal from the current pose. */
    public double getGoalBearingDeg() {
        Shotplanner.HiveTarget hive = getHiveTarget();
        double dx = hive.openingX - OdoMetry.X() / 100;
        double dy = hive.openingY - OdoMetry.Y() / 100;
        double bearing = Math.toDegrees(Math.atan2(dy, dx)) - OdoMetry.Heading();
        bearing %= 360.0;
        if (bearing > 180) bearing -= 360;
        if (bearing <= -180) bearing += 360;
        return bearing;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public boolean flywheelReady() {
        return targetRPM > 0
                && Math.abs(RPM - targetRPM) <= targetRPM * ShooterTuning.flywheelReadyPercent / 100.0;
    }

    @Override
    public void execute() {
        long start = System.nanoTime();
        applyTuning();
        encoder.UpdatePosition();

        double dt = Math.min(0.1, loopTimer.seconds());
        loopTimer.reset();

        inputFilter.update(
                OdoMetry.getXVelocity() / 100,
                OdoMetry.getYVelocity() / 100,
                OdoMetry.getHeadingVelocityDeg(),
                encoder.getVelocity(),
                OdoMetry.getXAcceleration() / 100,
                OdoMetry.getYAcceleration() / 100,
                OdoMetry.getHeadingAccelerationDeg(),
                dt
        );

        double rawRpm = shooterMotor.getVelocity() / ShooterTuning.flywheelTicksPerRev * 60;
        double filter = Math.max(0.01, Math.min(1, ShooterTuning.flywheelRpmFilter));
        RPM += filter * (rawRpm - RPM);

        if (targeting && turretDebugMode == TurretDebugMode.CLOSED_LOOP) {
            turretDebugMode = TurretDebugMode.OFF;
        }

        if (turretDebugMode == TurretDebugMode.MANUAL) {
            if (lastTurretDebugMode != TurretDebugMode.MANUAL) {
                turretController.stop();
            }
            double cmd = Math.max(0, Math.min(1, 0.5 + manualTurretPower));
            turretServo1.setPosition(manualServoSelect == ServoSelect.SERVO2_ONLY ? 0.5 : cmd);
            turretServo2.setPosition(manualServoSelect == ServoSelect.SERVO1_ONLY ? 0.5 : cmd);
            stopFlywheel();

        } else if (turretDebugMode == TurretDebugMode.CLOSED_LOOP) {
            stopFlywheel();

        } else if (targeting) {
            if (!wasTargeting || plannerTimer.milliseconds() >= ShooterTuning.plannerPeriodMs) {
                plannerTimer.reset();
                runShotPlanner();
            }

            flywheelPower = flywheel.update(targetRPM, RPM, dt);
            shooterMotor.update(flywheelPower);

        } else {
            stopFlywheel();
            turretController.stop();
            solutionHold.reset();
        }

        wasTargeting = targeting && turretDebugMode == TurretDebugMode.OFF;
        lastTurretDebugMode = turretDebugMode;

        if (turretDebugMode != TurretDebugMode.MANUAL) {
            turretController.update();
        }

        ((OpModeEX) getOpMode()).profiler.recordDuration(LoopProfiler.TURRET, System.nanoTime() - start);
    }

    private void stopFlywheel() {
        targetRPM = 0;
        flywheelPower = 0;
        flywheel.reset();
        shooterMotor.update(0);
    }

    private void runShotPlanner() {
        Shotplanner.RobotState robot = new Shotplanner.RobotState(
                OdoMetry.X()/100,
                OdoMetry.Y()/100,

                inputFilter.velocityX(),
                inputFilter.velocityY(),

                inputFilter.accelerationX(),
                inputFilter.accelerationY(),

                OdoMetry.Heading(),
                inputFilter.headingRate(),
                inputFilter.headingAcceleration(),

                inputFilter.turretRate()
        );

        long plannerStart = System.nanoTime();
        Shotplanner.ShotSolution shot = shotPlanner.calculateShot(
                robot,
                getHiveTarget(),
                ShooterTuning.plannerDelaySeconds
        );
        plannerMs = (System.nanoTime() - plannerStart) / 1e6;
        lastShot = shot;
        lastRobotState = robot;

        Shotplanner.ShotSolution act = solutionHold.apply(shot);

        if (act.reachable
                && Double.isFinite(act.turretAngleDeg)
                && Double.isFinite(act.launchAngleDeg)
                && Double.isFinite(act.flywheelRPM)) {
            targetRPM = act.flywheelRPM;
            turretController.setTarget(act.turretAngleDeg, 1.0, false);
            setHoodDegrees(act.launchAngleDeg);
        } else {
            turretController.setTarget(getGoalBearingDeg(), 1.0, false);
        }
    }
}
