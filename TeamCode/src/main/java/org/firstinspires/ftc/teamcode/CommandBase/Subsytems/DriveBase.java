package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;
import org.firstinspires.ftc.teamcode.CommandBase.LoopProfiler;

import dev.weaponboy.nexus_command_base.Commands.Command;
import dev.weaponboy.nexus_command_base.Commands.LambdaCommand;
import dev.weaponboy.nexus_command_base.Hardware.MotorEx;
import dev.weaponboy.nexus_command_base.Hardware.ServoDegrees;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;

public class DriveBase extends SubSystem {

    public MotorEx LF = new MotorEx();
    public MotorEx RF = new MotorEx();
    public MotorEx RB = new MotorEx();
    public MotorEx LB = new MotorEx();
    public TouchSensor intakeSensor;
    Servo pto1;
    Servo pto2;
    ServoDegrees baseServo = new ServoDegrees();

    public double speed = 1;
    public boolean engage = false;
    public boolean lift = false;

    PIDController headingPID = new PIDController(0.025, 0, 0.0003);

    double vertikal;
    double turn;
    double strafe;
    public boolean tele = true;

    public DriveBase(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, driveCommand);
    }

    @Override
    public void init() {
        LF.initMotor("LF", getOpMode().hardwareMap);
        RF.initMotor("RF", getOpMode().hardwareMap);
        LB.initMotor("LB", getOpMode().hardwareMap);
        RB.initMotor("RB", getOpMode().hardwareMap);
        pto1 = getOpMode().hardwareMap.get(Servo.class, "pto1");
        pto2 = getOpMode().hardwareMap.get(Servo.class, "pto2");
        baseServo.initServo("base", getOpMode().hardwareMap);
        baseServo.setRange(180);
        baseServo.setDirection(Servo.Direction.REVERSE);

        intakeSensor = getOpMode().hardwareMap.get(TouchSensor.class, "intakeSensor");

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double headingLock(double headingError, boolean on) {
        if (on) {
            turn = headingPID.calculate(headingError);
        }

        return headingPID.calculate(headingError);
    }

    @Override
    public void execute() {
        long start = System.nanoTime();
        executeEX();

        if (engage) {
            pto1.setPosition(0.64);
            pto2.setPosition(0.32);
            baseServo.setPosition(35);
        } else {
            pto1.setPosition(0.5);
            pto2.setPosition(0.5);
            baseServo.setPosition(0);
        }
        ((OpModeEX) getOpMode()).profiler.recordDuration(LoopProfiler.DRIVE_BASE, System.nanoTime() - start);
    }

    public Command drivePowers(double vertical, double turn, double strafe) {
        this.turn = turn;
        this.strafe = strafe;
        this.vertikal = vertical;

        return driveCommand;

    }

    public Command drivePowers(RobotPower power) {
        this.turn = power.getPivot();
        this.strafe = -power.getVertical();
        this.vertikal = -power.getHorizontal();

        return driveCommand;
    }

    LambdaCommand driveCommand = new LambdaCommand(
            () -> {
            },
            () -> {
                long start = System.nanoTime();
                double denominator = Math.max(1.0, Math.abs(vertikal) + Math.abs(strafe) + Math.abs(turn));

                double lfT = ((vertikal - strafe - turn) / denominator) * speed;
                double rfT = ((vertikal + strafe + turn) / denominator) * speed;
                double lbT = ((vertikal + strafe - turn) / denominator) * speed;
                double rbT = ((vertikal - strafe + turn) / denominator) * speed;

                if (!engage) {
                    LF.update(lfT);
                    RF.update(rfT);
                    LB.update(lbT);
                    RB.update(rbT);
                } else {
                    LF.update(lfT);
                    RF.update(rfT);
                    LB.update(0);
                    RB.update(0);
                }
                ((OpModeEX) getOpMode()).profiler.recordDuration(LoopProfiler.DRIVE_BASE, System.nanoTime() - start);
            },
            () -> true);

    public void setAll(double power) {
        LF.update(power);
        RF.update(power);
        LB.update(power);
        RB.update(power);
    }

    public void driveFieldCentric(double drive, double strafe, double turn, double robotHeading) {
        // Convert degrees → radians (most common IMU method in FTC)
        double headingRadians = Math.toRadians(robotHeading);

        // Now rotate the translation vector opposite to the robot's heading
        double rotX = strafe * Math.cos(-headingRadians) - drive * Math.sin(-headingRadians);
        double rotY = strafe * Math.sin(-headingRadians) + drive * Math.cos(-headingRadians);
        if (Math.abs( rotX) < 0.2){
            rotX = 0;
        }
        drivePowers(rotY, turn, rotX);
    }

}