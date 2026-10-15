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

    // Direct-drive (bare motor) protection: never plug-reverse a wheel that is
    // still travelling fast. Above reverseBrakeSpeed a reversal is replaced by
    // a 0-power command (motor BRAKE, which draws far less current than
    // plugging); below it the reversal is allowed but slew-limited.
    public boolean protectReversal = true;
    public double reverseBrakeSpeed = 25;   // cm/s, robot speed from odometry
    public double reverseSlewPerSec = 6;    // power/s while crossing/leaving zero
    private final double[] applied = new double[4];   // LF, RF, LB, RB last output
    private long lastProtectNanos = 0;

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

    public Command drivePowers(double vertical, double strafe, double turn) {
        this.turn = -turn;
        this.strafe = -strafe;
        this.vertical = vertical;

        return driveCommand;

    }

    public Command drivePowers(RobotPower power) {
        this.turn = -power.getPivot();
        this.strafe = -power.getHorizontal();
        this.vertical = power.getVertical();

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

                double m0, m1, m2, m3;
                if (!engage) {
                    m0 = -lfT; m1 = rfT; m2 = -lbT; m3 = rbT;
                } else {
                    m0 = lfT; m1 = rfT; m2 = 0; m3 = 0;
                }
                if (protectReversal) {
                    long now = System.nanoTime();
                    double dt = lastProtectNanos == 0 ? 0.02 : Math.min(0.1, (now - lastProtectNanos) / 1e9);
                    lastProtectNanos = now;
                    OpModeEX op = (OpModeEX) getOpMode();
                    double v = Math.hypot(op.odometry.getXVelocity(), op.odometry.getYVelocity())
                            + Math.abs(op.odometry.getHVelocity()) * 20;  // spin contribution (~wheel radius from centre, cm)
                    m0 = protect(0, m0, dt, v);
                    m1 = protect(1, m1, dt, v);
                    m2 = protect(2, m2, dt, v);
                    m3 = protect(3, m3, dt, v);
                }
                LF.update(m0);
                RF.update(m1);
                LB.update(m2);
                RB.update(m3);
                ((OpModeEX) getOpMode()).profiler.recordDuration(LoopProfiler.DRIVE_BASE, System.nanoTime() - start);
            },
            () -> true);



    /** Reversal guard for one wheel. Returns the power to actually send. */
    private double protect(int i, double target, double dt, double speedCmS) {
        double cur = applied[i];
        boolean reversing = cur * target < 0;
        double out = target;

        if (reversing && speedCmS > reverseBrakeSpeed) {
            out = 0;                                   // brake instead of plug-reversing
        } else if (target != 0 && (Math.signum(cur) != Math.signum(target) || Math.abs(cur) < 0.1)) {
            // crossing zero or leaving rest: limit how fast power builds
            double maxStep = reverseSlewPerSec * dt;
            out = cur + Math.max(-maxStep, Math.min(maxStep, target - cur));
        }
        applied[i] = out;
        return out;
    }

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