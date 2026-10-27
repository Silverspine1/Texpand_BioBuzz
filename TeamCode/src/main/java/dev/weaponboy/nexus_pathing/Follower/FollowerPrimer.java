package dev.weaponboy.nexus_pathing.Follower;

import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

/**
 * Keeps the Follower's PID "previous error" current while those PIDs are idle.
 * The library's D term is kd * (error - previousError) and previousError is 0
 * after setPath() (or stale from the last time that PID ran), so the first loop
 * a PID is used produces a one-loop power spike - a visible twitch on entering
 * the 3 cm end zone, and at the start of a turn. Lives in the library's package
 * to reach its package-private PIDs.
 */
public final class FollowerPrimer {

    private static final double END_ZONE = 3;   // matches Follower.followPathAuto

    private FollowerPrimer() {}

    /** Call every loop, BEFORE followPathAuto. */
    public static void prime(Follower f, double H, double X, double Y) {
        if (!f.pathFinished) {
            Vector2D e = f.getErrorToPointOnPath(X, Y);
            if (Math.hypot(e.getX(), e.getY()) >= END_ZONE) {
                double ex, ey;
                if (f.disableGlobalFollowing) {
                    ex = e.getX();
                    ey = e.getY();
                } else {
                    ex = e.getY() * Math.sin(Math.toRadians(H)) + e.getX() * Math.cos(Math.toRadians(H));
                    ey = e.getY() * Math.cos(Math.toRadians(H)) - e.getX() * Math.sin(Math.toRadians(H));
                }
                f.finalAdjustmentXPID.calculate(ex);
                f.finalAdjustmentYPID.calculate(ey);
            }
        }
    }

    /** Call once right after setPath(), before the first followPathAuto. */
    public static void primeHeading(Follower f, double targetHeading, double H) {
        double rot = targetHeading - H;
        if (rot < -180) rot += 360;
        else if (rot > 180) rot -= 360;
        f.fastHeadingPID.calculate(-rot);
        f.slowHeadingPID.calculate(-rot);
    }
}
