package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Drivebase;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.CommandBase.LoopProfiler;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

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


    public double speed = 1;
    public boolean engage = false;

    PIDController headingPID = new PIDController(0.025, 0, 0.0003);

    double vertical;
    double turn;
    double strafe;

    public DriveBase(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, driveCommand);
    }

    enum alliance_side {
        blue,
        red
    }
    alliance_side Alliance = alliance_side.blue;

    @Override
    public void init() {
        LF.initMotor("LF", getOpMode().hardwareMap);
        RF.initMotor("RF", getOpMode().hardwareMap);
        LB.initMotor("LB", getOpMode().hardwareMap);
        RB.initMotor("RB", getOpMode().hardwareMap);



    }



    @Override
    public void execute() {
        executeEX();
    }

    public Command drivePowers(double vertical, double turn, double strafe) {
        this.turn = turn;
        this.strafe = strafe;
        this.vertical = vertical;

        return driveCommand;

    }

    public Command drivePowers(RobotPower power) {
        this.turn = power.getPivot();
        this.strafe = -power.getVertical();
        this.vertical = -power.getHorizontal();

        return driveCommand;
    }

    LambdaCommand driveCommand = new LambdaCommand(
            () -> {
            },
            () -> {
                long start = System.nanoTime();
                double denominator = Math.max(1.0, Math.abs(vertical) + Math.abs(strafe) + Math.abs(turn));

                double lfT = ((vertical - strafe - turn) / denominator) * speed;
                double rfT = ((vertical + strafe + turn) / denominator) * speed;
                double lbT = ((vertical + strafe - turn) / denominator) * speed;
                double rbT = ((vertical - strafe + turn) / denominator) * speed;

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



    public void driveFieldCentric(double drive, double strafe, double turn, double robotHeading) {
        double headingOffset ;
        if (Alliance == alliance_side.blue) {
            headingOffset = -270 ;
        } else {
             headingOffset = - 90 ;
        }
        // Convert degrees → radians (most common IMU method in FTC)
        double headingRadians = Math.toRadians(robotHeading + headingOffset);

        // Now rotate the translation vector opposite to the robot's heading
        double rotX = strafe * Math.cos(-headingRadians) - drive * Math.sin(-headingRadians);
        double rotY = strafe * Math.sin(-headingRadians) + drive * Math.cos(-headingRadians);
        if (Math.abs( rotX) < 0.2){
            rotX = 0;
        }
        drivePowers(rotY, turn, rotX);
    }

}