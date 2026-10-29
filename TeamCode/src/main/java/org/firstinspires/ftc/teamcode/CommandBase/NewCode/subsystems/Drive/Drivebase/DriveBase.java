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

    // Direct-drive (bare motor) protection. Per wheel, current ~ (power - speed)
    // as fractions of stall current / free speed. While a wheel is being slowed
    // (current against its motion) that braking current ramps up at
    // brakeCurrentRate and is capped at brakeCurrentMax - so no shock, even
    // from a 0-power BRAKE at full speed, yet the wheel decelerates at constant
    // current (much faster than passive BRAKE, whose current dies with speed).
    // Driving current and a stopped wheel are not limited.
    public boolean protectReversal = true;
    public double brakeCurrentMax = 0.8;     // fraction of stall current
    public double brakeCurrentRate = 15;     // stall fractions per second
    private final double[] brakeI = new double[4];
    private long lastShapeNanos = 0;
    // Full-power speeds used to estimate wheel speed from odometry. Lower = more
    // conservative. PathTuner overwrites these with measured values.
    public double fullSpeedFwd = 180;       // cm/s
    public double fullSpeedStr = 140;       // cm/s
    public double fullSpeedTurn = 400;      // deg/s
    public boolean odoVelocityRobotFrame = false;
    private final double[] wheelSpeed = new double[4];   // LF, RF, LB, RB fraction of full speed

    public double vertical;

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
        this.strafe = strafe;
        this.vertical = vertical;

        return driveCommand;

    }

    public Command drivePowers(RobotPower power) {
        this.turn = -power.getPivot();
        this.strafe = power.getHorizontal();
        this.vertical = power.getVertical();

        return driveCommand;
    }

    LambdaCommand driveCommand = new LambdaCommand(
            () -> {
            },
            () -> {
                long start = System.nanoTime();
                double denominator = Math.max(1.0, Math.abs(vertical) + Math.abs(strafe) + Math.abs(turn));

                double lfT = ((vertical + strafe + turn) / denominator) * speed;
                double rfT = ((vertical - strafe - turn) / denominator) * speed;
                double lbT = ((vertical - strafe + turn) / denominator) * speed;
                double rbT = ((vertical + strafe - turn) / denominator) * speed;

                long now = System.nanoTime();
                double dt = lastShapeNanos == 0 ? 0.02 : Math.min(0.1, (now - lastShapeNanos) / 1e9);
                lastShapeNanos = now;
                if (protectReversal) {
                    estimateWheelSpeeds();
                    lfT = shapeBraking(0, lfT, wheelSpeed[0], dt);
                    rfT = shapeBraking(1, rfT, wheelSpeed[1], dt);
                    lbT = shapeBraking(2, lbT, wheelSpeed[2], dt);
                    rbT = shapeBraking(3, rbT, wheelSpeed[3], dt);
                }

                double m0, m1, m2, m3;
                if (!engage) {
                    m0 = -lfT; m1 = rfT; m2 = -lbT; m3 = rbT;
                } else {
                    m0 = lfT; m1 = rfT; m2 = 0; m3 = 0;
                }
                LF.update(m0);
                RF.update(m1);
                LB.update(m2);
                RB.update(m3);
                ((OpModeEX) getOpMode()).profiler.recordDuration(LoopProfiler.DRIVE_BASE, System.nanoTime() - start);
            },
            () -> true);



    /** Wheel speeds in the same (pre sign-flip) space as lfT..rbT, as a fraction of full speed. */
    private void estimateWheelSpeeds() {
        OpModeEX op = (OpModeEX) getOpMode();
        double vx = op.odometry.getXVelocity();
        double vy = op.odometry.getYVelocity();
        double fwd, str;
        if (odoVelocityRobotFrame) {
            fwd = vx;
            str = vy;
        } else {
            double h = Math.toRadians(op.odometry.Heading());
            fwd = vy * Math.sin(h) + vx * Math.cos(h);
            str = vy * Math.cos(h) - vx * Math.sin(h);
        }
        double wf = fwd / fullSpeedFwd;
        double ws = str / fullSpeedStr;
        double wt = op.odometry.getHeadingVelocityDeg() / fullSpeedTurn;
        wheelSpeed[0] = wf + ws + wt;
        wheelSpeed[1] = wf - ws - wt;
        wheelSpeed[2] = wf - ws + wt;
        wheelSpeed[3] = wf + ws - wt;
    }

    private double shapeBraking(int k, double cmd, double w, double dt) {
        double i = cmd - w;
        if (Math.abs(w) < 0.02 || i * w >= 0) {
            brakeI[k] = 0;
            return cmd;
        }
        double mag = Math.min(Math.abs(i), Math.min(brakeCurrentMax, brakeI[k] + brakeCurrentRate * dt));
        brakeI[k] = mag;
        return w - Math.signum(w) * mag;
    }

    /** Largest |wheel speed| fraction right now (for telemetry / tuning). */
    public double maxWheelSpeedFraction() {
        double m = 0;
        for (double w : wheelSpeed) m = Math.max(m, Math.abs(w));
        return m;
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