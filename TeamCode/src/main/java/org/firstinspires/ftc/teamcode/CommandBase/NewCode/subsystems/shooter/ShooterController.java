package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Localisation.Odometry;
import org.firstinspires.ftc.teamcode.CommandBase.LoopProfiler;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_command_base.Hardware.MotorEx;
import dev.weaponboy.nexus_command_base.Hardware.ServoDegrees;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;


public class ShooterController extends SubSystem {

    static final double HOOD_MIN_LAUNCH_DEG = 10;
    static final double HOOD_MAX_LAUNCH_DEG = 45;
    static final double HOOD_SERVO_RANGE_DEG = 253;
    static final double PLANNER_PERIOD_MS = 50;

    Shotplanner shotPlanner = new Shotplanner(plannerConfig());
    ElapsedTime plannerTimer = new ElapsedTime();
    boolean wasTargeting = false;
    public AxonEncoder encoder;
    Odometry OdoMetry;

    Shotplanner.BlueHiveSide blueHiveSide = Shotplanner.BlueHiveSide.AUDIENCE;
    ServoDegrees Hood = new ServoDegrees();
    Servo turretServo1;
    Servo turretServo2;
    SetTurretAngle turretController;

    public MotorEx shooterMotor = new MotorEx();
    double targetRPM = 0;
    public double RPM;
    public boolean targeting = false;

    public PIDController shootPID = new PIDController(0.3, 0.000, 0.01);

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
        turretDebugMode = TurretDebugMode.CLOSED_LOOP;
        turretController.setTarget(angleDeg, 1.0, false);
    }

    private static Shotplanner.Config plannerConfig() {
        Shotplanner.Config config = new Shotplanner.Config();
        config.minLaunchAngleDeg = Math.max(config.minLaunchAngleDeg, HOOD_MIN_LAUNCH_DEG);
        config.maxLaunchAngleDeg = Math.min(config.maxLaunchAngleDeg, HOOD_MAX_LAUNCH_DEG);
        return config;
    }

    public ShooterController(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, returnDefaultCommand());
        this.OdoMetry = opModeEX.odometry;
    }

    @Override
    public void init() {
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
    }

    public void setHoodDegrees(double theta) {
        double servoPos = (theta - HOOD_MIN_LAUNCH_DEG)
                / (HOOD_MAX_LAUNCH_DEG - HOOD_MIN_LAUNCH_DEG) * HOOD_SERVO_RANGE_DEG;
        servoPos = Math.max(0, Math.min(HOOD_SERVO_RANGE_DEG, servoPos));
        Hood.setPosition(servoPos);
    }

    @Override
    public void execute() {
        long start = System.nanoTime();
        encoder.UpdatePosition();

        RPM = ((shooterMotor.getVelocity() / 33.6) * 60);

        if (turretDebugMode == TurretDebugMode.MANUAL) {
            if (lastTurretDebugMode != TurretDebugMode.MANUAL) {
                turretController.stop();
            }
            double cmd = Math.max(0, Math.min(1, 0.5 + manualTurretPower));
            turretServo1.setPosition(manualServoSelect == ServoSelect.SERVO2_ONLY ? 0.5 : cmd);
            turretServo2.setPosition(manualServoSelect == ServoSelect.SERVO1_ONLY ? 0.5 : cmd);
            shooterMotor.update(0);

        } else if (turretDebugMode == TurretDebugMode.CLOSED_LOOP) {
            shooterMotor.update(0);

        } else if (targeting) {
            if (!wasTargeting || plannerTimer.milliseconds() >= PLANNER_PERIOD_MS) {
                plannerTimer.reset();
                runShotPlanner();
            }

            double power = shootPID.calculate(targetRPM, RPM);
            if (Double.isNaN(power)) {
                power = 0;
            }
            shooterMotor.update(Math.max(0, Math.min(1, power)));

        } else {
            targetRPM = 0;
            shooterMotor.update(0);
            turretController.stop();
        }

        wasTargeting = targeting && turretDebugMode == TurretDebugMode.OFF;
        lastTurretDebugMode = turretDebugMode;

        if (turretDebugMode != TurretDebugMode.MANUAL) {
            turretController.update();
        }

        ((OpModeEX) getOpMode()).profiler.recordDuration(LoopProfiler.TURRET, System.nanoTime() - start);
    }

    private void runShotPlanner() {
        Shotplanner.RobotState robot = new Shotplanner.RobotState(
                OdoMetry.X()/100,
                OdoMetry.Y()/100,

                OdoMetry.getXVelocity()/100,
                OdoMetry.getYVelocity()/100,

                OdoMetry.getXAcceleration()/100,
                OdoMetry.getYAcceleration()/100,

                // Heading() is 360 - raw heading, so the raw rad/s rates are negated.
                OdoMetry.Heading(),
                -Math.toDegrees(OdoMetry.getHeadingVelocity()),
                -Math.toDegrees(OdoMetry.getHAcceleration()),

                encoder.getVelocity()
        );

        long plannerStart = System.nanoTime();
        Shotplanner.ShotSolution shot = shotPlanner.calculateShot(
                robot,
                Shotplanner.getBlueHiveTarget(blueHiveSide),
                0.100
        );
        plannerMs = (System.nanoTime() - plannerStart) / 1e6;
        lastShot = shot;
        lastRobotState = robot;

        if (shot.reachable
                && Double.isFinite(shot.turretAngleDeg)
                && Double.isFinite(shot.launchAngleDeg)
                && Double.isFinite(shot.flywheelRPM)) {
            targetRPM = shot.flywheelRPM;
            turretController.setTarget(shot.turretAngleDeg, 1.0, false);
            setHoodDegrees(shot.launchAngleDeg);
        }
    }
}
