package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Drivebase;

/**
 * Field-centric drive, pure Java (no SDK, no allocation): driver stick -> robot-frame {@code vertical} / {@code strafe}.
 *
 * <p>Frame = the Odometry (Java) frame = the fixed RED team frame (see ShotFrame): X from the audience wall to the back wall,
 * Y toward the right seen from the audience, heading grows from +X toward +Y, the robot's right is (-sin h, cos h). A positive
 * {@code vertical} drives along the heading (+h), a positive {@code strafe} to the robot's right (the same axes as
 * DriveBase.estimateWheelSpeeds and the Follower use), whichever end of the robot the driver treats as the front.
 *
 * <p>The driver stands at his own alliance wall looking across the field: red (Y = 0 wall) looks along +Y with his right hand
 * toward -X; blue (Y = W wall) looks along -Y with his right hand toward +X. Stick UP = toward the opposing alliance wall,
 * stick RIGHT = the driver's right, whatever the robot's heading. {@code offsetDeg} rotates that stick frame in the sense of
 * increasing heading (from +X toward +Y) for a driver who stands somewhere else: +90 on red = stick up drives toward the
 * audience wall (-X).
 */
public final class FieldDrive {

    /** Outputs of {@link #solve}. */
    public double vertical, strafe;
    /** The field direction the stick asks for, Java frame (unit stick = unit vector), for the displays. */
    public double fieldX, fieldY;

    /**
     * @param stickUp      +1 = stick pushed away from the driver
     * @param stickRight   +1 = stick pushed to the driver's right
     * @param headingDeg   Odometry.Heading()
     * @param blue         alliance (decides which wall the driver stands at)
     * @param offsetDeg    rotation of the stick frame, deg (0 = driver at his alliance wall)
     */
    public void solve(double stickUp, double stickRight, double headingDeg, boolean blue, double offsetDeg) {
        // driver's forward / right as Java-frame unit vectors
        double fwdX = 0.0, fwdY = blue ? -1.0 : 1.0;
        double rgtX = blue ? 1.0 : -1.0, rgtY = 0.0;
        double a = Math.toRadians(offsetDeg), ca = Math.cos(a), sa = Math.sin(a);
        double fx = fwdX * ca - fwdY * sa, fy = fwdX * sa + fwdY * ca;
        double rx = rgtX * ca - rgtY * sa, ry = rgtX * sa + rgtY * ca;
        fieldX = stickUp * fx + stickRight * rx;
        fieldY = stickUp * fy + stickRight * ry;
        double h = Math.toRadians(headingDeg), ch = Math.cos(h), sh = Math.sin(h);
        vertical = fieldX * ch + fieldY * sh;
        strafe = -fieldX * sh + fieldY * ch;
    }
}
