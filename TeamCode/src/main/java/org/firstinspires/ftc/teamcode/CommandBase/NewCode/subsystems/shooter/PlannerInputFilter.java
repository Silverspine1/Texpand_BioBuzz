package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

/**
 * Cleans the motion inputs to the shot planner. The planner is very sensitive to them (a 1 cm/s velocity error moves
 * the turret command by degrees on a steep shot), and odometry acceleration is a differentiated velocity, which is
 * mostly noise, so by default it is not used.
 */
public class PlannerInputFilter {

    public double timeConstantSeconds = 0.06;
    public double velocityDeadbandMps = 0.02;
    public double headingRateDeadbandDps = 2.0;
    public double turretRateDeadbandDps = 5.0;
    public boolean useAcceleration = false;

    private boolean initialised = false;
    private double velocityX, velocityY, headingRate, turretRate;
    private double accelerationX, accelerationY, headingAcceleration;

    public void reset() {
        initialised = false;
    }

    public void update(double vx, double vy, double headingRateDps, double turretRateDps,
                       double ax, double ay, double headingAccelDps2, double dt) {
        if (!initialised) {
            velocityX = vx;
            velocityY = vy;
            headingRate = headingRateDps;
            turretRate = turretRateDps;
            initialised = true;
        } else {
            double alpha = timeConstantSeconds <= 0 || dt <= 0 ? 1 : 1 - Math.exp(-dt / timeConstantSeconds);
            velocityX += alpha * (vx - velocityX);
            velocityY += alpha * (vy - velocityY);
            headingRate += alpha * (headingRateDps - headingRate);
            turretRate += alpha * (turretRateDps - turretRate);
        }

        accelerationX = useAcceleration ? ax : 0;
        accelerationY = useAcceleration ? ay : 0;
        headingAcceleration = useAcceleration ? headingAccelDps2 : 0;
    }

    public double velocityX() { return softDeadband(velocityX, velocityDeadbandMps); }
    public double velocityY() { return softDeadband(velocityY, velocityDeadbandMps); }
    public double headingRate() { return softDeadband(headingRate, headingRateDeadbandDps); }
    public double turretRate() { return softDeadband(turretRate, turretRateDeadbandDps); }
    public double accelerationX() { return accelerationX; }
    public double accelerationY() { return accelerationY; }
    public double headingAcceleration() { return headingAcceleration; }

    /** Shrinks small values to zero without a jump at the edge of the band. */
    private static double softDeadband(double value, double band) {
        double magnitude = Math.abs(value) - band;
        return magnitude <= 0 ? 0 : Math.signum(value) * magnitude;
    }
}
