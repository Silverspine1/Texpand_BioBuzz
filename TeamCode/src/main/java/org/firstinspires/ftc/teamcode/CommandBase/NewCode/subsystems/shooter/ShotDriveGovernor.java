package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

/**
 * Shot-aware yaw limiter: while the driver holds the feed button, the rotation stick is shaped so the turret can keep up.
 * Pure Java (no SDK, no allocation); tested in tools/shot_java_parity/ShotParity.java against the Python bench
 * ({@code tools/method_plugins/govern_math.py}, experiment {@code gT70} in docs/SHOT_METHODS_EXPERIMENTS.md in the sim repo).
 *
 * <p>Why: the turret has to counter-rotate the chassis, and it moves at 300 deg/s at most with a ~45 ms dead time. A chassis
 * yaw rate the turret cannot follow is the biggest cause of missed shots while turning (sim, measured profile: a 64 % per-ball
 * make rate over a mixed set of moves became 78 %, and turning scenarios 87 % against 89 % standing still, on seeds the limit
 * was not tuned on).
 *
 * <p>How: the solver is probed ({@link ReducedShotController#turretRateProbe}) for the rate the turret COMMAND would move at,
 * {@code a} from the translation the sticks ask for and {@code b} per deg/s of yaw. The yaw rate {@code w} is allowed while
 * {@code |a + b*w| <= budget}. The stick is only ever REDUCED (same sign, never more than the driver asked for, 0 when the
 * window excludes it); translation is never touched.
 */
public final class ShotDriveGovernor {

    /** Master switch (ShotControlTuning.yawGovernorEnabled). */
    public boolean enabled = true;
    /** Turret command-rate budget [deg/s]: about 70 % of the turret's 300 deg/s (leave room for its lag). */
    public double budgetDps = 210.0;
    /** Drive model (PathingValues): speed per unit power ABOVE the breakaway power ks, response time constant. */
    public double spFwd = 260.0, spStr = 200.0, spTurn = 600.0;     // cm/s, cm/s, deg/s
    public double ksFwd = 0.06, ksStr = 0.08, ksTurn = 0.08;
    public double tauS = 0.15;
    /** Probe horizon [s] and probe yaw rate [deg/s]. */
    public double lookS = 0.10, probeYawDps = 100.0;

    // outputs, for the display and the stick intent
    /** The stick's command velocity, Java field frame [m/s] (computed every call). */
    public double cmdVxMps, cmdVyMps;
    public boolean active, limited;
    /** Turret command rate from translation (deg/s) and per deg/s of yaw, the budget used, the yaw rate asked / allowed [deg/s]. */
    public double a, b, budget, yawCmdDps, yawOutDps;
    public String state = "off";

    private final double[] ab = new double[2];

    /** Signed speed for a stick/power value: speedPerPower * (|p| - ks), 0 inside the breakaway band. */
    public static double speedFromPower(double p, double speedPerPower, double ks) {
        double m = Math.abs(p) - ks;
        return m <= 0.0 ? 0.0 : Math.signum(p) * m * speedPerPower;
    }

    /** Inverse of {@link #speedFromPower} for a nonzero speed. */
    public static double powerFromSpeed(double speed, double speedPerPower, double ks) {
        return speed == 0.0 ? 0.0 : Math.signum(speed) * (Math.abs(speed) / Math.max(1e-9, speedPerPower) + ks);
    }

    /** Yaw rates w for which |a + b*w| <= budget, as {lo, hi}; null when yaw does not move the turret command (|b| tiny): no limit. */
    public static double[] yawWindow(double a, double b, double budget) {
        if (Math.abs(b) <= 0.2) return null;
        double l = (-budget - a) / b, h = (budget - a) / b;
        return l <= h ? new double[]{l, h} : new double[]{h, l};
    }

    /** The governed yaw rate: only ever REDUCES the driver's yaw (result between 0 and the stick, same sign); 0 when the window excludes
     *  the stick on the far side of zero. */
    public static double clampYaw(double wCmd, double a, double b, double budget) {
        double[] win = yawWindow(a, b, budget);
        if (win == null) return wCmd;
        double res = wCmd < win[0] ? win[0] : (wCmd > win[1] ? win[1] : wCmd);
        return (res * wCmd >= 0.0 && Math.abs(res) <= Math.abs(wCmd)) ? res : 0.0;
    }

    /**
     * @param r         the shot controller (probed for the turret command rate)
     * @param fireHeld  the driver holds the feed button and the reduced controller is the command source
     * @param turn      rotation stick (+ = heading increasing, as {@code DriveBase.drivePowers})
     * @param vertical  forward power and strafe power (+ = right) as {@code DriveBase.drivePowers} gets them (after the field-centric transform)
     * @param headingDeg Odometry.Heading()
     * @return the rotation stick to drive with
     */
    public double apply(ReducedShotController r, boolean fireHeld, double turn, double vertical, double strafe, double headingDeg) {
        // the command velocity of the translation sticks (always: the turret lead can use it too)
        double vF = speedFromPower(vertical, spFwd, ksFwd), vS = speedFromPower(strafe, spStr, ksStr);
        double h = Math.toRadians(headingDeg), ch = Math.cos(h), sh = Math.sin(h);
        cmdVxMps = (vF * ch - vS * sh) * 0.01;
        cmdVyMps = (vF * sh + vS * ch) * 0.01;
        active = false;
        limited = false;
        yawCmdDps = speedFromPower(turn, spTurn, ksTurn);
        yawOutDps = yawCmdDps;
        if (!enabled) { state = "off"; return turn; }
        if (!fireHeld) { state = "idle (feed not held)"; return turn; }
        if (!r.turretRateProbe(cmdVxMps, cmdVyMps, tauS, lookS, probeYawDps, ab)) { state = "no solve yet"; return turn; }
        active = true;
        a = ab[0];
        b = ab[1];
        budget = budgetDps;
        double w = clampYaw(yawCmdDps, a, b, budget);
        yawOutDps = w;
        if (w == yawCmdDps) { state = "ACTIVE (no limit needed)"; return turn; }
        limited = true;
        state = "ACTIVE: yaw limited";
        return powerFromSpeed(w, spTurn, ksTurn);
    }

    public String statusLine() {
        if (!active) return state;
        return String.format(java.util.Locale.US, "%s  yaw %.0f -> %.0f deg/s  (turret rate: translation %+.0f, per yaw %+.2f, budget %.0f)",
                state, yawCmdDps, yawOutDps, a, b, budget);
    }
}
