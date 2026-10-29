package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.Follower;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathingUtility.RobotPower;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.RobotUtilities.RobotConfig;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.RobotUtilities.Vector2D;

/**
 * Path follower (Nexus API, new control law).
 *
 * Each loop:
 *  1. Predict the robot one actuation delay ahead (pos + vel * (delay + dt)).
 *  2. Target speed along the path = min of: top speed for this direction,
 *     the corner limit, sqrt(2*decel*r) (latest point it can still brake on
 *     time) and kEnd*r (gentle final approach), where r is the arc length to
 *     the end plus the planned overshoot.
 *  3. Commanded velocity = that speed along the tangent + kCross * error
 *     back onto the path. Commanded accel = the profile's braking slope along
 *     the actual motion + centripetal accel on curves.
 *  4. Power per robot axis from the drive model:
 *        v/speedPerPower + tau*a/speedPerPower + kS + velGain*(v - vMeasured)/speedPerPower
 *  5. Once past the end (or stopped next to it) it settles on the end point,
 *     and outputs 0 inside holdT.
 * Heading: fast PD far from the target, soft PD inside 3 deg (back to fast
 * beyond 6), 0 turn inside holdH, breakaway power added outside it. D is per
 * second.
 */
public class Follower extends FollowerBaseMethods {

    private static final double SOFT_IN = 3, SOFT_OUT = 6;    // deg
    private static final double SETTLE_SPEED = 3, SETTLE_DIST = 5;
    private static final double HOLD_SPEED = 8;
    private static final double KS_SPEED = 3;                // cm/s - kS fades in over this commanded speed

    /** Time source (nanoseconds) - swap it to run the follower in a simulation. */
    public interface Clock { long nanos(); }
    public static Clock clock = System::nanoTime;

    private long lastNanos = 0;
    private boolean settling = false;
    private boolean softHeading = false;
    private boolean fresh = true;
    private double prevRot = Double.NaN;

    // telemetry
    private double lastTargetSpeed = 0, lastRemaining = 0;

    public Follower() {
        this(new RobotConfig());
    }

    public Follower(RobotConfig customConfig) {
        config = customConfig;
    }

    public RobotConfig getConfig() {
        return config;
    }

    public void robotHeadingBackwards(boolean robotHeadingBackwards) {
        this.robotHeadingBackwards = robotHeadingBackwards;
        offset = robotHeadingBackwards ? 180 : 0;
    }

    public void disableGlobalFollowing(boolean disableGlobalFollowing) {
        this.disableGlobalFollowing = disableGlobalFollowing;
    }

    public void setHeadingOffset(double headingOffset) {
        this.headingOffset = headingOffset;
    }

    public void setHeadingLookAheadDistance(int headingLookAheadDistance) {
        this.headingLookAheadDistance = headingLookAheadDistance;
    }

    /** Forces the soft heading PD everywhere. */
    public void slowerHeading(boolean slowerHeading) {
        this.slowerHeading = slowerHeading;
    }

    public void usePathHeadings(boolean usePathHeadings) {
        this.usePathHeadings = usePathHeadings;
    }

    public void setPath(PathBuilder path) {
        pathFinished = false;
        forceStop = false;
        currentIndex = 0;
        createNewPathOperator(path);
        settling = false;
        heldXY = false;
        prevRot = Double.NaN;
        lastNanos = 0;
        fresh = true;
    }

    public RobotPower followPathAuto(double targetHeading, double H, double X, double Y, double XV, double YV) {
        long now = clock.nanos();
        double gap = lastNanos == 0 ? 0.02 : (now - lastNanos) / 1e9;
        if (gap > 0.2) prevRot = Double.NaN;
        double dt = clamp(gap, 0.002, 0.1);
        lastNanos = now;
        if (!finite(H) || !finite(X) || !finite(Y)) return new RobotPower(0, 0, 0);
        if (!finite(XV)) XV = 0;
        if (!finite(YV)) YV = 0;
        velocityX = XV;
        velocityY = YV;
        robotPositionVector.set(X, Y);

        if (pathoperator == null || pathoperator.size() < 2) {
            return new RobotPower(0, 0, getTurnPower(targetHeading, H, dt));
        }

        double lead = config.delay + dt;
        double px = X + XV * lead, py = Y + YV * lead;
        int idx = fresh ? pathoperator.resetClosest(px, py) : pathoperator.closestIndex(px, py);
        fresh = false;
        currentIndex = idx;

        double h = Math.toRadians(H);
        double sin = Math.sin(h), cos = Math.cos(h);

        double vx = 0, vy = 0, ax = 0, ay = 0;
        boolean translate = !(pathFinished && !holdPosition);

        Vector2D end = pathoperator.getPointOnFollowable(pathoperator.getLastPoint());
        double ex = end.getX() - X, ey = end.getY() - Y;
        double dist = Math.hypot(ex, ey);
        double speed = Math.hypot(XV, YV);
        int last = pathoperator.size() - 1;

        if (translate && !settling) {
            double along = ex * pathoperator.tangentX(last) + ey * pathoperator.tangentY(last);
            boolean nearEnd = pathoperator.remaining(idx) < SETTLE_DIST + config.overshoot + 10;
            if (pathFinished || (nearEnd && along <= 0) || (nearEnd && speed < SETTLE_SPEED && dist < SETTLE_DIST)) {
                settling = true;
            }
        }

        if (translate && settling) {
            if (heldXY && dist > 2 * config.holdT) heldXY = false;
            if (!heldXY && dist <= config.holdT && speed < HOLD_SPEED) heldXY = true;
            if (heldXY) translate = false;
        }

        if (translate && settling) {
            // point mode: straight at the end point, no planned overshoot
            double dx = end.getX() - px, dy = end.getY() - py;
            double r = Math.hypot(dx, dy);
            if (r > 1e-6) {
                double ux = dx / r, uy = dy / r;
                double[] va = profile(r, ux, uy, Double.MAX_VALUE, XV * ux + YV * uy, sin, cos);
                vx = ux * va[0];
                vy = uy * va[0];
                ax = ux * va[1];
                ay = uy * va[1];
            }
            lastRemaining = r;
        } else if (translate) {
            double tx = pathoperator.tangentX(idx), ty = pathoperator.tangentY(idx);
            Vector2D p = pathoperator.getPointOnFollowable(idx);
            double ox = px - p.getX(), oy = py - p.getY();
            double along = ox * tx + oy * ty;
            double sRem = pathoperator.remaining(idx) - along;
            double r = sRem + config.overshoot;
            double vAlong = XV * tx + YV * ty;
            double[] va = profile(r, tx, ty, pathoperator.cornerSpeed(idx), vAlong, sin, cos);

            double cx = -(ox - along * tx), cy = -(oy - along * ty);
            double cmax = dirMaxSpeed(cx, cy, sin, cos);
            double cMag = Math.hypot(cx, cy) * config.kCross;
            double cs = cMag > cmax && cMag > 1e-9 ? cmax / cMag : 1;

            vx = va[0] * tx + config.kCross * cx * cs;
            vy = va[0] * ty + config.kCross * cy * cs;
            double an = pathoperator.curvature(idx) * vAlong * vAlong;
            ax = va[1] * tx - an * ty;
            ay = va[1] * ty + an * tx;
            lastRemaining = sRem;
        }

        double pf = 0, ps = 0;
        lastTargetSpeed = Math.hypot(vx, vy);
        if (translate) {
            double vf, vs, af, as, mf, ms;
            if (disableGlobalFollowing) {
                vf = vx; vs = vy; af = ax; as = ay; mf = XV; ms = YV;
            } else {
                vf = vy * sin + vx * cos;
                vs = vy * cos - vx * sin;
                af = ay * sin + ax * cos;
                as = ay * cos - ax * sin;
                mf = YV * sin + XV * cos;
                ms = YV * cos - XV * sin;
            }
            pf = axisPower(vf, af, mf, config.speedPerPowerFwd, config.tauFwd, config.ksFwd, config.velGainFwd);
            ps = axisPower(vs, as, ms, config.speedPerPowerStr, config.tauStr, config.ksStr, config.velGainStr);
            double sum = Math.abs(pf) + Math.abs(ps);
            if (sum > config.powerLimit && sum > 1e-9) {
                pf *= config.powerLimit / sum;
                ps *= config.powerLimit / sum;
            }
        }

        clipLookAheadIndex(idx + headingLookAheadDistance);
        double hdg = targetHeading;
        if (usePathHeadings) {
            hdg = calculateTargetHeading();
            if (hdg > 360) hdg -= 360;
        }
        double turn = getTurnPower(hdg, H, dt);
        if (!finite(pf) || !finite(ps) || !finite(turn)) return new RobotPower(0, 0, 0);
        return new RobotPower(pf, ps, turn);
    }

    private static boolean finite(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }

    /** {target speed, target accel} along (ux, uy) for distance-to-go r (signed). */
    private double[] profile(double r, double ux, double uy, double corner, double vMeas,
                             double sin, double cos) {
        double sgn = r >= 0 ? 1 : -1;
        double rr = Math.abs(r);
        double decel = dirBlend(ux, uy, sin, cos, config.MAX_X_ACCELERATION(), config.MAX_Y_ACCELERATION());
        double kEnd = dirBlend(ux, uy, sin, cos, config.kEndFwd, config.kEndStr);
        double vTop = Math.min(dirMaxSpeed(ux, uy, sin, cos), corner);
        double vBrake = Math.sqrt(2 * decel * rr);
        double vTail = kEnd * rr;
        double v = Math.min(vTop, Math.min(vBrake, vTail));
        double dvdr = 0;
        if (v == vTail) dvdr = kEnd;
        else if (v == vBrake && vBrake > 1e-6) dvdr = decel / vBrake;
        double a = -dvdr * Math.max(0, vMeas * sgn);
        return new double[]{sgn * v, sgn * a};
    }

    private double axisPower(double v, double a, double vMeas, double spp, double tau, double ks, double velGain) {
        spp = Math.max(1, spp);
        return v / spp + tau * a / spp + ks * Math.tanh(v / KS_SPEED) + velGain * (v - vMeas) / spp;
    }

    /** Top speed in field direction (dx, dy): mecanum share |f|/Vf + |s|/Vs <= 1. */
    private double dirMaxSpeed(double dx, double dy, double sin, double cos) {
        return dirBlend(dx, dy, sin, cos, config.MAX_X_VELOCITY(), config.MAX_Y_VELOCITY());
    }

    /** A forward / strafe capability for a field direction: 1 / (|f|/fwd + |s|/str). */
    private double dirBlend(double dx, double dy, double sin, double cos, double fwd, double str) {
        fwd = Math.max(1e-3, fwd);
        str = Math.max(1e-3, str);
        double len = Math.hypot(dx, dy);
        if (len < 1e-9) return Math.min(fwd, str);
        double f, s;
        if (disableGlobalFollowing) {
            f = Math.abs(dx) / len;
            s = Math.abs(dy) / len;
        } else {
            f = Math.abs(dy * sin + dx * cos) / len;
            s = Math.abs(dy * cos - dx * sin) / len;
        }
        return 1 / (f / fwd + s / str);
    }

    public double getTurnPower(double targetHeading, double currentHeading, double dt) {
        double rot = wrap180(targetHeading - currentHeading);
        double dRot = Double.isNaN(prevRot) ? 0 : wrap180(rot - prevRot) / dt;
        prevRot = rot;
        double a = Math.abs(rot);
        if (softHeading) {
            if (a > SOFT_OUT) softHeading = false;
        } else if (a < SOFT_IN) {
            softHeading = true;
        }
        boolean soft = softHeading || slowerHeading;
        double p = soft ? config.headingSlowP : config.headingFastP;
        double d = soft ? config.headingSlowD : config.headingFastD;
        double turn = -(p * rot + d * dRot);
        if (soft && a <= config.holdH) return 0;
        if (a > config.holdH) turn -= Math.signum(rot) * config.ksTurn;
        return turn;
    }

    public boolean isHoldPosition() {
        return holdPosition;
    }

    public void holdPositionAtPathEnd(boolean holdPosition) {
        this.holdPosition = holdPosition;
    }

    /** Kept for API compatibility; friction is handled by the kS feedforward. */
    public void headingErrorToOvercomeFriction(boolean useHeadingFrictionError) {
        this.useHeadingFrictionError = useHeadingFrictionError;
    }

    // ---------------- telemetry ----------------

    public boolean isSoftHeading() { return softHeading || slowerHeading; }
    public boolean isSettling() { return settling; }
    public boolean isHolding() { return heldXY; }
    public double getTargetSpeed() { return lastTargetSpeed; }
    public double getRemaining() { return lastRemaining; }

    private static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    private static double wrap180(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }
}
