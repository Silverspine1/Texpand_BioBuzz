package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Localisation.Odometry;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_command_base.Hardware.MotorEx;
import dev.weaponboy.nexus_command_base.Hardware.ServoDegrees;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;


public class ShooterController extends SubSystem {

    Shotplanner shotPlanner = new Shotplanner(new Shotplanner.Config());
    private AxonEncoder encoder;
    Odometry OdoMetry;

    Shotplanner.BlueHiveSide blueHiveSide = Shotplanner.BlueHiveSide.AUDIENCE;
    ServoDegrees Hood = new ServoDegrees();
    Servo turretServo1;
    Servo turretServo2;
    SetTurretAngle turretController;

    public MotorEx shooterMotor = new MotorEx();
    double targetRPM = 0;
    double RPM;
    public boolean targeting = false;

    public PIDController shootPID = new PIDController(0.3, 0.000, 0.01);

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
        double servoPos = (theta - 10) / (45 - 10) * 253.0;
        servoPos = Math.max(0, Math.min(253, servoPos));
        Hood.setPosition(servoPos);
    }

    @Override
    public void execute() {

        Shotplanner.RobotState robot = new Shotplanner.RobotState(
                OdoMetry.X()/100,
                OdoMetry.Y()/100,

                OdoMetry.getXVelocity()/100,
                OdoMetry.getYVelocity()/100,

                OdoMetry.getXAcceleration()/100,
                OdoMetry.getYAcceleration(),

                OdoMetry.Heading(),
                OdoMetry.getHeadingVelocity(),
                OdoMetry.getHAcceleration(),

                encoder.getVelocity()
        );

        Shotplanner.HiveTarget hive =
                Shotplanner.getBlueHiveTarget(blueHiveSide);

        Shotplanner.ShotSolution shot =
                shotPlanner.calculateShot(
                        robot,
                        hive,
                        0.100
                );
        RPM = ((shooterMotor.getVelocity() / 33.6) * 60);
        if (targeting) {
            shooterMotor.update(Math.max(0, shootPID.calculate(shot.flywheelRPM, RPM)));
            if (shot.reachable){
                turretController.setTarget(shot.turretAngleDeg, 1.0, false);
                setHoodDegrees(shot.launchAngleDeg);
            }
        }else {
            shooterMotor.update(0);
            turretController.stop();
        }

        turretController.update();
    }
}
