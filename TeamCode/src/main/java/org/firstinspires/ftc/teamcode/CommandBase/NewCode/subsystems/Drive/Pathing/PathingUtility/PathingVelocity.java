package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathingUtility;

public class PathingVelocity {

    private double xVelocity;
    private double yVelocity;

    public PathingVelocity(double xVelocity, double yVelocity){
        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
    }

    public PathingVelocity() {
        this(0, 0);
    }

    public PathingVelocity set(double xVelocity, double yVelocity) {
        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
        return this;
    }

    public double getXVelocity(){
        return xVelocity;
    }

    public double getYVelocity(){
        return yVelocity;
    }

    private PathingVelocity add(double xVelocity, double yVelocity) {
        return new PathingVelocity(this.xVelocity + xVelocity, this.yVelocity + yVelocity);
    }

}
