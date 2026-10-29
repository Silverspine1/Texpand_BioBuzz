package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.Follower;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathLog;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathingUtility.RobotPower;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.RobotUtilities.RobotConfig;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.RobotUtilities.Vector2D;

/**
 * Path follower (Nexus API, new control law).
 *
 * Each loop:
 *  1. Predict the robot one actuation delay ahead: speed from the drive
 *     model and the commands already sent but not yet acting (a Smith
 *     predictor), position from that speed. Without the speed part the
 *     controller kept braking after its braking had already done the job,
 *     nearly stopped short and crawled in.
 *  2. Target speed along the path = min of: top speed for this direction,
 *     the corner limit, sqrt(2*decel*r) (latest point it can still brake on
 *     time) and kEnd*r (gentle final approach), where r is the arc length to
 *     the end plus the planned overshoot.
 *  3. Commanded velocity = that speed along the tangent + kCross * error
 *     back onto the path. Commanded accel = the profile's braking slope along
 *     the actual motion + centripetal accel on curves.
 *  4. Power per robot axis from the drive model:
 *        v/speedPerPower + tau*a/speedPerPower + velGain*(v - vMeasured)/speedPerPower
 *     plus breakaway power along the commanded DIRECTION (a small sideways
 *     correction while driving forward gets a small share, not a full kS -
 *     per-axis kS acted like a relay and shook the robot sideways).
 *  5. Once past the end (or stopped next to it) it settles on the end point,
 *     and outputs 0 inside holdT.
 * Heading uses the same idea: target turn rate = min(top rate,
 * sqrt(2*turnDecel*r), kEndH*r) on the predicted heading error plus a planned
 * overshoot (faded out near the target), power from the turn model, 0 turn
 * once inside holdH and slow. Turn breakaway power only while not translating
 * (rolling wheels need none). Full rate until it has to brake - no slow
 * exponential crawl like a PD.
 */
public class Follower extends FollowerBaseMethods {

    private static final double SETTLE_SPEED = 3, SETTLE_DIST = 5;
    private static final double HOLD_SPEED = 8;
    private static final int HOLD_LOOPS = 2;                 // slow this many loops in a row before holding
    private static final double HOLD_RELEASE = 1.0;          // cm past holdT before a hold lets go
    private static final double OVERSHOOT_FADE = 15;         // cm - planned overshoot fades out over the last this
    // Final approach is a position loop through the actuation delay: above
    // kEnd * (delay + loop) ~ 0.7 it bounces around the target (sim, and the
    // robot logs). kEnd is capped there whatever the config says.
    public static final double K_END_DELAY_LIMIT = 0.7;
    private static final double KS_SPEED = 3;                // cm/s - kS fades in over this commanded speed
    private static final double TURN_KS_SPEED = 20;          // cm/s - turn kS gone above this robot speed
    private static final double HOLD_RATE = 8;               // deg/s
    private static final double HOLD_RELEASE_H = 1.5;        // deg past holdH before a heading hold lets go
    private static final double OVERSHOOT_FADE_H = 15;       // deg
    private static final double KS_RATE = 10;                // deg/s - turn kS fades in over this commanded rate

    /** Time source (nanoseconds) - swap it to run the follower in a simulation. */
    public interface Clock { long nanos(); }
    public static Clock clock = System::nanoTime;

    private long lastNanos = 0;
    private double loopDt = 0.02;
    private boolean settling = false;
    private int slowLoops = 0;
    private double ksFade = 1;
    private boolean heldH = false;
    private int slowLoopsH = 0;
    private boolean fresh = true;
    private double prevH = Double.NaN, hvEst = 0;

    // commands sent (robot frame: fwd, strafe, turn with + = H increasing)
    private static final int HIST = 32;
    private final double[] histT = new double[HIST], histF = new double[HIST], histS = new double[HIST], histW = new double[HIST];
    private int histN = 0, histHead = 0;

    // telemetry / log
    private double lastTargetSpeed = 0, lastRemaining = 0;
    private int lastLimiter = 0;            // 0 top speed, 1 corner, 2 braking sqrt, 3 end tail
    private double lastVTarget = 0, lastRot = 0, lastDRot = 0;
    private double robotSpeed = 0;

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

    /** Halves the heading final-approach stiffness. */
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
        slowLoops = 0;
        heldH = false;
        slowLoopsH = 0;
        // heading-rate estimate, loop timing and the in-flight command history
        // describe the ROBOT, not the path: keep them, so re-planning mid-drive
        // (vision collect re-plans every 0.25 s) does not jolt the predictor.
        // They reset themselves after a gap in calls.
        fresh = true;
        PathLog.newRun();
    }

    /** Heading rate estimated from successive headings - prefer the overload
     *  that takes the odometry's heading velocity. */
    public RobotPower followPathAuto(double targetHeading, double H, double X, double Y, double XV, double YV) {
        return followPathAuto(targetHeading, H, X, Y, XV, YV, Double.NaN);
    }

    /** HV: heading rate, deg/s, same sign as H increasing (odometry.getHeadingVelocityDeg()). */
    public RobotPower followPathAuto(double targetHeading, double H, double X, double Y, double XV, double YV, double HV) {
        long now = clock.nanos();
        double gap = lastNanos == 0 ? 0.02 : (now - lastNanos) / 1e9;
        if (gap > 0.2) {
            prevH = Double.NaN;
            hvEst = 0;
            histN = 0;
        }
        double dt = clamp(gap, 0.002, 0.1);
        loopDt += 0.1 * (dt - loopDt);
        lastNanos = now;
        if (!finite(H) || !finite(X) || !finite(Y)) return new RobotPower(0, 0, 0);
        if (!finite(XV)) XV = 0;
        if (!finite(YV)) YV = 0;
        if (!finite(HV)) {
            double raw = Double.isNaN(prevH) ? 0 : wrap180(H - prevH) / dt;
            hvEst += 0.5 * (raw - hvEst);
            HV = hvEst;
        }
        prevH = H;
        velocityX = XV;
        velocityY = YV;
        robotPositionVector.set(X, Y);

        double nowS = now / 1e9;
        double h = Math.toRadians(H);
        double sin = Math.sin(h), cos = Math.cos(h);
        double mf, ms;
        if (disableGlobalFollowing) {
            mf = XV;
            ms = YV;
        } else {
            mf = YV * sin + XV * cos;
            ms = YV * cos - XV * sin;
        }
        double speed = Math.hypot(XV, YV);
        robotSpeed = speed;

        // Smith predictor: speeds when the next command starts acting
        double turnKs = config.ksTurn * clamp(1 - speed / TURN_KS_SPEED, 0, 1);
        double hvP = predictSpeed(HV, inFlight(2, nowS, config.delayTurn), config.speedPerPowerTurn, config.tauTurn, turnKs, config.delayTurn);
        double leadH = config.delayTurn + dt;
        double hP = H + 0.5 * (HV + hvP) * leadH;

        if (pathoperator == null || pathoperator.size() < 2) {
            double turn = turnPower(targetHeading, H, hP, hvP);
            remember(nowS, 0, 0, -turn);
            return new RobotPower(0, 0, turn);
        }

        double mfP = predictSpeed(mf, inFlight(0, nowS, config.delay), config.speedPerPowerFwd, config.tauFwd, config.ksFwd, config.delay);
        double msP = predictSpeed(ms, inFlight(1, nowS, config.delay), config.speedPerPowerStr, config.tauStr, config.ksStr, config.delay);
        double vxP, vyP;
        if (disableGlobalFollowing) {
            vxP = mfP;
            vyP = msP;
        } else {
            vxP = mfP * cos - msP * sin;
            vyP = mfP * sin + msP * cos;
        }
        double lead = config.delay + dt;
        double px = X + 0.5 * (XV + vxP) * lead, py = Y + 0.5 * (YV + vyP) * lead;
        int idx = fresh ? pathoperator.resetClosest(px, py) : pathoperator.closestIndex(px, py);
        fresh = false;
        currentIndex = idx;

        double vx = 0, vy = 0, ax = 0, ay = 0;
        boolean translate = !(pathFinished && !holdPosition);
        lastVTarget = 0;
        lastLimiter = -1;

        Vector2D end = pathoperator.getPointOnFollowable(pathoperator.getLastPoint());
        double ex = end.getX() - X, ey = end.getY() - Y;
        double dist = Math.hypot(ex, ey);
        int last = pathoperator.size() - 1;

        if (translate && !settling) {
            double along = ex * pathoperator.tangentX(last) + ey * pathoperator.tangentY(last);
            boolean nearEnd = pathoperator.remaining(idx) < SETTLE_DIST + config.overshoot + 10;
            if (pathFinished || (nearEnd && along <= 0) || (nearEnd && speed < SETTLE_SPEED && dist < SETTLE_DIST)) {
                settling = true;
            }
        }

        if (translate && settling) {
            // a speed zero-crossing in a bounce is not "stopped": need a few slow loops
            slowLoops = speed < HOLD_SPEED ? slowLoops + 1 : 0;
            if (heldXY && dist > config.holdT + HOLD_RELEASE) heldXY = false;
            if (!heldXY && dist <= config.holdT && slowLoops >= HOLD_LOOPS) heldXY = true;
            if (heldXY) translate = false;
        }

        ksFade = 1;
        if (translate && settling) {
            // point mode: straight at the end point, no planned overshoot.
            // Breakaway power fades out approaching the hold zone - at full
            // strength it flips direction every time the robot crosses the
            // target and the robot hunts around it.
            ksFade = clamp((dist - 0.5 * config.holdT) / Math.max(0.1, config.holdT), 0, 1);
            double dx = end.getX() - px, dy = end.getY() - py;
            double r = Math.hypot(dx, dy);
            if (r > 1e-6) {
                double ux = dx / r, uy = dy / r;
                double[] va = profile(r, ux, uy, Double.MAX_VALUE, vxP * ux + vyP * uy, sin, cos);
                lastVTarget = va[0];
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
            // planned overshoot moves the braking point later, but fades out
            // near the end so the target speed AT the end is 0 - otherwise the
            // hand-over to settling reverses a still-moving command (a jolt)
            double r = sRem + config.overshoot * clamp(sRem / OVERSHOOT_FADE, 0, 1);
            double vAlong = vxP * tx + vyP * ty;
            double[] va = profile(r, tx, ty, pathoperator.cornerSpeed(idx), vAlong, sin, cos);
            lastVTarget = va[0];

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
        double vf = 0, vs = 0, af = 0, as = 0;
        if (translate) {
            if (disableGlobalFollowing) {
                vf = vx; vs = vy; af = ax; as = ay;
            } else {
                vf = vy * sin + vx * cos;
                vs = vy * cos - vx * sin;
                af = ay * sin + ax * cos;
                as = ay * cos - ax * sin;
            }
            pf = axisPower(vf, af, mfP, config.speedPerPowerFwd, config.tauFwd, config.velGainFwd);
            ps = axisPower(vs, as, msP, config.speedPerPowerStr, config.tauStr, config.velGainStr);
            double vmag = Math.hypot(vf, vs);
            if (vmag > 1e-6) {
                double k = ksFade * Math.tanh(vmag / KS_SPEED) / vmag;
                pf += config.ksFwd * vf * k;
                ps += config.ksStr * vs * k;
            }
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
        double turn = turnPower(hdg, H, hP, hvP);
        if (!finite(pf) || !finite(ps) || !finite(turn)) {
            pf = 0;
            ps = 0;
            turn = 0;
        }
        remember(nowS, pf, ps, -turn);
        if (PathLog.active()) {
            double etx = pathoperator.tangentX(last), ety = pathoperator.tangentY(last);
            int mode = !translate && heldXY ? 2 : !(translate || settling) ? 3 : settling ? 1 : 0;
            PathLog.row(0, 0, dt, mode, lastLimiter, idx,
                    X, Y, H, XV, YV,
                    end.getX(), end.getY(), dist, ex * etx + ey * ety, -ex * ety + ey * etx, lastRemaining,
                    lastVTarget, vf, vs, af, as, mf, ms,
                    pf, ps, turn, hdg, lastRot, lastDRot, heldH ? 1 : 0);
        }
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
        double kEnd = Math.min(dirBlend(ux, uy, sin, cos, config.kEndFwd, config.kEndStr), kEndLimit(config.delay, loopDt));
        double vTop = Math.min(dirMaxSpeed(ux, uy, sin, cos), corner);
        double vBrake = Math.sqrt(2 * decel * rr);
        double vTail = kEnd * rr;
        double v = Math.min(vTop, Math.min(vBrake, vTail));
        lastLimiter = v == vTail ? 3 : v == vBrake ? 2 : vTop < dirMaxSpeed(ux, uy, sin, cos) ? 1 : 0;
        double dvdr = 0;
        if (v == vTail) dvdr = kEnd;
        else if (v == vBrake && vBrake > 1e-6) dvdr = decel / vBrake;
        double a = -dvdr * Math.min(v, Math.max(0, vMeas * sgn));
        return new double[]{sgn * v, sgn * a};
    }

    private void remember(double t, double f, double s, double w) {
        histHead = (histHead + 1) % HIST;
        histT[histHead] = t;
        histF[histHead] = f;
        histS[histHead] = s;
        histW[histHead] = w;
        histN = Math.min(HIST, histN + 1);
    }

    /** Mean command (axis 0 fwd, 1 strafe, 2 turn) sent in the last window seconds - still in flight. */
    private double inFlight(int axis, double t, double window) {
        double sum = 0;
        int n = 0;
        for (int k = 0; k < histN; k++) {
            int i = (histHead - k + HIST) % HIST;
            if (t - histT[i] > window + 1e-6) break;
            sum += axis == 0 ? histF[i] : axis == 1 ? histS[i] : histW[i];
            n++;
        }
        return n == 0 ? 0 : sum / n;
    }

    /** First-order drive model: speed after time T under power p (with friction). */
    private static double predictSpeed(double v, double p, double spp, double tau, double ks, double T) {
        double dir = Math.abs(v) > 3 ? Math.signum(v) : Math.signum(p);
        if (Math.abs(v) <= 3 && Math.abs(p) < ks) return v;           // static friction holds it
        double vss = Math.max(1, spp) * (p - ks * dir);
        if (Math.abs(v) <= 3 && vss * dir < 0) vss = 0;
        return vss + (v - vss) * Math.exp(-T / Math.max(0.02, tau));
    }

    /** Highest stable final-approach stiffness for this delay and loop time. */
    public static double kEndLimit(double delay, double loopDt) {
        return K_END_DELAY_LIMIT / Math.max(0.02, delay + loopDt);
    }

    private double axisPower(double v, double a, double vMeas, double spp, double tau, double velGain) {
        spp = Math.max(1, spp);
        return v / spp + tau * a / spp + velGain * (v - vMeas) / spp;
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

    /** Pivot power (Nexus sign: negative makes H increase), no prediction. HV in deg/s. */
    public double getTurnPower(double targetHeading, double currentHeading, double HV, double dt) {
        return turnPower(targetHeading, currentHeading, currentHeading + HV * (config.delayTurn + dt), HV);
    }

    /** hP / HV: predicted heading and heading rate when this command starts acting. */
    private double turnPower(double targetHeading, double currentHeading, double hP, double HV) {
        double rotNow = wrap180(targetHeading - currentHeading);
        double rot = wrap180(targetHeading - hP);
        lastRot = rotNow;
        lastDRot = HV;
        double aNow = Math.abs(rotNow);

        slowLoopsH = Math.abs(HV) < HOLD_RATE ? slowLoopsH + 1 : 0;
        if (heldH && aNow > config.holdH + HOLD_RELEASE_H) heldH = false;
        if (!heldH && aNow <= config.holdH && slowLoopsH >= HOLD_LOOPS) heldH = true;
        if (heldH) return 0;

        double sgn = rot >= 0 ? 1 : -1;
        double a = Math.abs(rot);
        double r = a + config.overshootH * clamp(a / OVERSHOOT_FADE_H, 0, 1);
        double decel = Math.max(50, config.brakeDecelTurn * config.decelMargin);
        // same motors and loop as translation, so the larger measured delay is the safe one
        double k = Math.min(config.kEndH * (slowerHeading ? 0.5 : 1), kEndLimit(Math.max(config.delayTurn, config.delay), loopDt));
        double spp = Math.max(1, config.speedPerPowerTurn);
        double wTop = spp * Math.max(0.05, config.powerLimit - config.ksTurn);
        double wBrake = Math.sqrt(2 * decel * r), wTail = k * r;
        double w = Math.min(wTop, Math.min(wBrake, wTail));
        double dwdr = w == wTail ? k : (w == wBrake && wBrake > 1e-6 ? decel / wBrake : 0);
        double wRef = sgn * w;
        // braking slope at the planned rate, not the actual one: when the robot
        // runs faster than the profile the actual-rate slope times a long turn
        // time constant over-brakes and it stops short (feedback covers the rest)
        double aRef = -sgn * dwdr * Math.min(w, Math.max(0, HV * sgn));
        double h = Math.max(0.1, config.holdH);
        double ksScale = clamp(1 - robotSpeed / TURN_KS_SPEED, 0, 1) * clamp((aNow - 0.5 * h) / h, 0, 1);
        double p = wRef / spp + config.tauTurn * aRef / spp + config.velGainH * (wRef - HV) / spp
                + config.ksTurn * Math.tanh(wRef / KS_RATE) * ksScale;
        p = clamp(p, -config.powerLimit, config.powerLimit);
        return -p;
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

    public boolean isHoldingHeading() { return heldH; }
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
