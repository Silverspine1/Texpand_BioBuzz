package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Localisation;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.CommandBase.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.CommandBase.GoBildaPinpointDriver.Register;
import org.firstinspires.ftc.teamcode.CommandBase.LoopProfiler;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_command_base.Commands.LambdaCommand;
import dev.weaponboy.nexus_command_base.Hardware.MotorEx;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;

public class Odometry extends SubSystem {

    //public Odometry(OpModeEX opModeEX) {registerSubsystem(opModeEX, returnDefaultCommand());}

    private final LambdaCommand resetPosition = new LambdaCommand(
            () -> {},
            () -> {

            },
            () -> false
    );



    enum TipeOfOdo {
        Threeweel,
        PintPoint
    }

    TipeOfOdo tipeOfOdo = TipeOfOdo.PintPoint;

    public void setOdoTipe(TipeOfOdo tipe){
        tipeOfOdo = tipe;
    }

    // Gobloda vearbles
    public GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;

    Register[] onlyPosition = {
            Register.X_POSITION,
            Register.Y_POSITION,
            Register.H_ORIENTATION,
            Register.X_VELOCITY,
            Register.Y_VELOCITY,
    };

   /* DcMotorEx leftPod;
    DcMotorEx rightPod;
    DcMotorEx backPod;*/

    public double X, Y, Heading, normilised;
    double startX, startY, startHeading;

    double XVelocity = 0;
    double YVelocity = 0;
    double HVelocity = 0;

    // Pinpoint acceleration and heading velocity
    // X/Y acceleration: cm/s²
    // Heading acceleration: rad/s²
    // Heading velocity: rad/s
    double XAcceleration = 0;
    double YAcceleration = 0;
    double HAcceleration = 0;
    double HeadingVelocity = 0;

    private long lastPinpointVelocitySampleNanos = 0;

    boolean resetAtStart = false;


    // Three well verables

    MotorEx leftPod = new MotorEx();
    MotorEx rightPod = new MotorEx();
    MotorEx backPod = new MotorEx();

    //double X, Y, Heading;
    //int startHeading;
    public double otherHeading;

    double lastRightPod, lastLeftPod, lastBackPod;
    double currentRightPod, currentLeftPod, currentBackPod;
    double rightPodPos, leftPodPos, backPodPos;

    double podTicks = 2000;
    double wheelRadius = 2.4;
    double trackWidth = 24.2;
    double backPodOffset = 9.4;

    double ticksPerCM = ((2.0 * Math.PI) * wheelRadius)/podTicks;
    double cmPerDegreeX = (double) (2) / 360;
    double cmPerDegreeY = ((2.0 * Math.PI) * backPodOffset) / 360;

    double currentXVelocity = 0;
    double currentYVelocity = 0;

    boolean sampleReset = true;
    boolean runningDistanceSensorReset = false;
    int resetCounter = 0;

    public Odometry(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, update);
    }

    public void startPosition(double X, double Y, int Heading) {
        if (tipeOfOdo == TipeOfOdo.PintPoint){
            this.startX = X;
            this.startY = Y;
            this.Heading = Heading;
        } else {
            this.X = X;
            this.Y = Y;
            this.startHeading = Heading;
            this.Heading = Math.toRadians(Heading);

        }
    }

    @Override
    public void init() {
        // int pint point
        odo = getOpMode().hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // --- Pinpoint I2C Optimization (Phase 2) ---
        // Only read what we actually use. Excluding LOOP_TIME, etc.
        odo.setBulkReadScope(
                Register.X_POSITION,
                Register.Y_POSITION,
                Register.H_ORIENTATION,
                Register.X_VELOCITY,
                Register.Y_VELOCITY,
                Register.H_VELOCITY);

        odo.setOffsets(120, -15, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();


        // int three well
        leftPod.initMotor("LF", getOpMode().hardwareMap);
        rightPod.initMotor("RF", getOpMode().hardwareMap);
        backPod.initMotor("RB", getOpMode().hardwareMap);
    }

    public double headingError(double targetHeading) {
        return Heading - targetHeading;
    }

    @Override
    public void execute() {

        if (tipeOfOdo == TipeOfOdo.PintPoint) {
            long start = System.nanoTime();
            executeEX();

            ((OpModeEX) getOpMode()).profiler.recordDuration(LoopProfiler.ODOMETRY, System.nanoTime() - start);
        } else {
            executeEX();
            updateVelocity();

//                System.out.println("execute odometry update line based");

            lastBackPod = currentBackPod;
            lastLeftPod = currentLeftPod;
            lastRightPod = currentRightPod;

            currentBackPod = -backPod.getCurrentPosition();
            currentLeftPod = -leftPod.getCurrentPosition();
            currentRightPod = -rightPod.getCurrentPosition();

            double deltaRight = currentRightPod - lastRightPod;
            double deltaLeft = currentLeftPod - lastLeftPod;
            double deltaBack = currentBackPod - lastBackPod;

            double deltaHeading = (ticksPerCM * (deltaRight - deltaLeft)) / (trackWidth+0.22);
            Heading += deltaHeading;

            if (Math.toDegrees(Heading) < 0){
                Heading = Math.toRadians(360 - Math.toDegrees(Heading));
            } else if (Math.toDegrees(Heading) > 360) {
                Heading = Math.toRadians(Math.toDegrees(Heading) - 360);
            }

            double deltaX = ((((deltaRight+deltaLeft)*ticksPerCM)/2)) + (Math.toDegrees(deltaHeading) * cmPerDegreeX);
//                double deltaX = ((((deltaRight+deltaLeft)*ticksPerCM)/2)) + (Math.toDegrees(deltaHeading) * cmPerDegreeX);
            double deltaY = (ticksPerCM * deltaBack) - (Math.toDegrees(deltaHeading) * cmPerDegreeY);

//                X += deltaX;
//                Y += deltaY;

            X += deltaX * Math.cos(Heading) - deltaY * Math.sin(Heading);
            Y += deltaX * Math.sin(Heading) + deltaY * Math.cos(Heading);

            //4165 back pod 180

//                updatePodReadings();
//                leftPod.update(0);
//                rightPod.update(0);
//                backPod.update(0);

//                System.out.println(Math.abs(rightPod.getTimeCompleted() - leftPod.getTimeCompleted())/1000000);


        }
    }

    public void updateVelocity(){
        double RRXError = ticksPerCM * ((-rightPod.getVelocity()+(-leftPod.getVelocity()))/2);
        double RRYError = ticksPerCM * -backPod.getVelocity();

//        currentXVelocity = RRXError;
//        currentYVelocity = RRYError;

        currentXVelocity = RRXError * Math.cos(Heading) - RRYError * Math.sin(Heading);
        currentYVelocity = RRXError * Math.sin(Heading) + RRYError * Math.cos(Heading);

    }

    public double X() {
        return X;
    }

    public double Y() {
        return Y;
    }

    public double Heading() {
        return 360 - Heading;
    }

    public double normiliased() {
        return normilised;
    }

    public double getYVelocity() {
        return YVelocity;
    }

    public double getXVelocity() {
        return XVelocity;
    }

    public double getHVelocity() {
        return HVelocity;
    }

    // Pinpoint acceleration getters

    public double getXAcceleration() {
        return XAcceleration;
    }

    public double getYAcceleration() {
        return YAcceleration;
    }

    public double getHAcceleration() {
        return HAcceleration;
    }

    public double getHeadingVelocity() {
        return HeadingVelocity;
    }

    public LambdaCommand update = new LambdaCommand(
            () -> {
            },
            () -> {

                odo.update();

                // Store previous velocities for Pinpoint acceleration calculation
                double previousXVelocity = XVelocity;
                double previousYVelocity = YVelocity;
                double previousHVelocity = HVelocity;

                XVelocity = -odo.getVelX(DistanceUnit.CM);
                YVelocity = odo.getVelY(DistanceUnit.CM);
                HVelocity = odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

                // Pinpoint-only acceleration and heading velocity
                if (tipeOfOdo == TipeOfOdo.PintPoint) {

                    long currentTimeNanos = System.nanoTime();

                    HeadingVelocity = HVelocity;

                    if (lastPinpointVelocitySampleNanos != 0) {

                        double deltaTime = (currentTimeNanos - lastPinpointVelocitySampleNanos) / 1_000_000_000.0;

                        if (deltaTime > 0) {
                            XAcceleration = (XVelocity - previousXVelocity) / deltaTime;
                            YAcceleration = (YVelocity - previousYVelocity) / deltaTime;
                            HAcceleration = (HVelocity - previousHVelocity) / deltaTime;
                        }
                    }

                    lastPinpointVelocitySampleNanos = currentTimeNanos;
                }

                // Cache heading reads (avoids redundant unit conversions)
                double rawDeg = odo.getHeading(AngleUnit.DEGREES);
                double rawRad = odo.getHeading(AngleUnit.RADIANS);

                Heading = startHeading + rawDeg;
                normilised = startHeading + rawRad;

                if (startHeading + rawDeg < 0) {
                    Heading = startHeading + rawDeg + 360;
                } else {
                    Heading = startHeading + rawDeg;
                }

                X = startX - odo.getPosX(DistanceUnit.CM);
                Y = startY + odo.getPosY(DistanceUnit.CM);
            },
            () -> false);

    /*public void offsetY(double offset) {
        Y += offset;
    }*/
}