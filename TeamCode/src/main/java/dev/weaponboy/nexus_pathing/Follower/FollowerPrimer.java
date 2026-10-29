package dev.weaponboy.nexus_pathing.Follower;

import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
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
    private static final double SLOW_IN = 3, SLOW_OUT = 6;   // deg, heading PID hand-over

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

    /**
     * One control step in place of followPathAuto (path headings off). The
     * library only ever runs its fast heading PID; this hands heading to the
     * soft slow PID near the target (with hysteresis, the idle PID kept primed
     * so the hand-over has no D kick), and outputs 0 turn inside holdTol so
     * stiction / gear lash cannot sustain a small limit cycle. Outside holdTol
     * turnKs (breakaway power) is added toward the target, so the soft,
     * delay-safe gains still finish the turn instead of stalling short.
     */
    public static RobotPower follow(Follower f, double targetHeading, double holdTol, double turnKs,
                                    double H, double X, double Y, double XV, double YV) {
        prime(f, H, X, Y);
        double rot = targetHeading - H;
        while (rot > 180) rot -= 360;
        while (rot < -180) rot += 360;
        double a = Math.abs(rot);
        if (f.slowerHeading) {
            if (a > SLOW_OUT) f.slowerHeading = false;
        } else if (a < SLOW_IN) {
            f.slowerHeading = true;
        }
        (f.slowerHeading ? f.fastHeadingPID : f.slowHeadingPID).calculate(-rot);
        RobotPower p = f.followPathAuto(targetHeading, H, X, Y, XV, YV);
        if (f.slowerHeading && a <= holdTol) p = new RobotPower(p.getVertical(), p.getHorizontal(), 0);
        else if (a > holdTol) p = new RobotPower(p.getVertical(), p.getHorizontal(), p.getPivot() - Math.signum(rot) * turnKs);
        return p;
    }

    public static boolean softHeading(Follower f) { return f.slowerHeading; }
}
