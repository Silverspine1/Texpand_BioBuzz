/*
package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Localisation;

import dev.weaponboy.nexus_command_base.Commands.LambdaCommand;
import dev.weaponboy.nexus_command_base.Hardware.MotorEx;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;


public class ODO extends SubSystem {



    MotorEx leftPod = new MotorEx();
    MotorEx rightPod = new MotorEx();
    MotorEx backPod = new MotorEx();

    double X, Y, Heading;
    int startHeading;
    public double otherHeading;

    double lastRightPod, lastLeftPod, lastBackPod;
    public double currentRightPod, currentLeftPod, currentBackPod;
    public double rightPodPos, leftPodPos, backPodPos;

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



    public void startPosition(double X, double Y, int Heading){
        this.X = X;
        this.Y = Y;
        this.startHeading = Heading;
        this.Heading = Math.toRadians(Heading);
    }

    @Override
    public void init() {
        leftPod.initMotor("leftPod", getOpMode().hardwareMap);
        rightPod.initMotor("rightPod", getOpMode().hardwareMap);
        backPod.initMotor("backPod", getOpMode().hardwareMap);

    }

    public double headingError(double targetHeading){
        return Heading-targetHeading;
    }

    @Override
    public void execute() {

        executeEX();
        updateVelocity();


    }

    public double X (){
        return X;
    }

    public double Y (){
        return Y;
    }

    public double Heading (){
        return Math.toDegrees(Heading);
    }

    public double getYVelocity(){
        return currentYVelocity;
    }

    public double getXVelocity(){
        return currentXVelocity;
    }

    public void updateVelocity(){
        double RRXError = ticksPerCM * ((-rightPod.getVelocity()+(-leftPod.getVelocity()))/2);
        double RRYError = ticksPerCM * -backPod.getVelocity();

//        currentXVelocity = RRXError;
//        currentYVelocity = RRYError;

        currentXVelocity = RRXError * Math.cos(Heading) - RRYError * Math.sin(Heading);
        currentYVelocity = RRXError * Math.sin(Heading) + RRYError * Math.cos(Heading);

    }



    public void offsetY(double offset){
        Y += offset;
    }

    public LambdaCommand updateLineBased = new LambdaCommand(
            () -> {},
            () -> {

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

            },
            () -> true
    );



    private final LambdaCommand resetPosition = new LambdaCommand(
            () -> {},
            () -> {

            },
            () -> false
    );

    private void updatePodReadings(){
//        leftPod.updatePosition();
//        rightPod.updatePosition();
//        backPod.updatePosition();
    }


}*/
