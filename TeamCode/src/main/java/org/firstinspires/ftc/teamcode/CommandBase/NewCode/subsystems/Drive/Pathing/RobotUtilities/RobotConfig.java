package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.RobotUtilities;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathingValues;

/**
 * Per-follower settings. {@code new RobotConfig()} copies the global
 * {@link PathingValues}; change a field or use a setter to override it for
 * one follower only. See PathingValues for units.
 */
public class RobotConfig {

    // drive model
    public double speedPerPowerFwd, speedPerPowerStr;
    public double tauFwd, tauStr;
    public double delay;
    public double ksFwd, ksStr, ksTurn;
    public double brakeDecelFwd, brakeDecelStr;

    // profile
    public double powerLimit;
    public double decelMargin;
    public double overshoot;
    public double kEndFwd, kEndStr;
    public double latAccel;
    public double maxSpeedFwd = 0, maxSpeedStr = 0;   // optional caps, cm/s (0 = none)

    // feedback
    public double velGainFwd, velGainStr;
    public double kCross;

    // holds
    public double holdT, holdH;

    // heading PD, D per second
    public double headingFastP, headingFastD, headingSlowP, headingSlowD;

    boolean logDebugging = false;

    public RobotConfig() {
        PathingValues.ensureLoaded();
        speedPerPowerFwd = PathingValues.SPEED_PER_POWER_FWD;
        speedPerPowerStr = PathingValues.SPEED_PER_POWER_STR;
        tauFwd = PathingValues.TAU_FWD;
        tauStr = PathingValues.TAU_STR;
        delay = PathingValues.DELAY;
        ksFwd = PathingValues.KS_FWD;
        ksStr = PathingValues.KS_STR;
        ksTurn = PathingValues.KS_TURN;
        brakeDecelFwd = PathingValues.BRAKE_DECEL_FWD;
        brakeDecelStr = PathingValues.BRAKE_DECEL_STR;
        powerLimit = PathingValues.POWER_LIMIT;
        decelMargin = PathingValues.DECEL_MARGIN;
        overshoot = PathingValues.OVERSHOOT;
        kEndFwd = PathingValues.K_END_FWD;
        kEndStr = PathingValues.K_END_STR;
        latAccel = PathingValues.LAT_ACCEL;
        velGainFwd = PathingValues.VEL_GAIN_FWD;
        velGainStr = PathingValues.VEL_GAIN_STR;
        kCross = PathingValues.K_CROSS;
        holdT = PathingValues.HOLD_T;
        holdH = PathingValues.HOLD_H;
        headingFastP = PathingValues.HEADING_FAST_P;
        headingFastD = PathingValues.HEADING_FAST_D;
        headingSlowP = PathingValues.HEADING_SLOW_P;
        headingSlowD = PathingValues.HEADING_SLOW_D;
    }

    /** Highest forward speed the profile may ask for, cm/s. */
    public double MAX_X_VELOCITY() {
        return cap(speedPerPowerFwd * Math.max(0.05, powerLimit - ksFwd), maxSpeedFwd);
    }

    /** Highest strafe speed the profile may ask for, cm/s. */
    public double MAX_Y_VELOCITY() {
        return cap(speedPerPowerStr * Math.max(0.05, powerLimit - ksStr), maxSpeedStr);
    }

    /** Forward deceleration the profile plans with, cm/s^2. */
    public double MAX_X_ACCELERATION() {
        return Math.max(50, brakeDecelFwd * decelMargin);
    }

    /** Strafe deceleration the profile plans with, cm/s^2. */
    public double MAX_Y_ACCELERATION() {
        return Math.max(50, brakeDecelStr * decelMargin);
    }

    private static double cap(double v, double c) {
        return c > 0 ? Math.min(v, c) : v;
    }

    public boolean logDebugging() {
        return logDebugging;
    }

    public void logDebugging(boolean logDebugging) {
        this.logDebugging = logDebugging;
    }

    public RobotConfig setFastHeadingPD(double p, double dPerSecond) {
        headingFastP = p;
        headingFastD = dPerSecond;
        return this;
    }

    public RobotConfig setSlowHeadingPD(double p, double dPerSecond) {
        headingSlowP = p;
        headingSlowD = dPerSecond;
        return this;
    }

    /** Speed caps (cm/s, 0 = none) and braking decelerations (cm/s^2, before DECEL_MARGIN). */
    public RobotConfig setRobotConstants(double maxXVelocity, double maxYVelocity, double maxXAcceleration, double maxYAcceleration) {
        maxSpeedFwd = maxXVelocity;
        maxSpeedStr = maxYVelocity;
        brakeDecelFwd = maxXAcceleration;
        brakeDecelStr = maxYAcceleration;
        return this;
    }

    public RobotConfig setPowerLimit(double powerLimit) {
        this.powerLimit = powerLimit;
        return this;
    }

    public RobotConfig setOvershoot(double cm) {
        overshoot = cm;
        return this;
    }

    public RobotConfig setHoldTolerance(double cm, double deg) {
        holdT = cm;
        holdH = deg;
        return this;
    }

    public RobotConfig setEndGains(double kFwd, double kStr) {
        kEndFwd = kFwd;
        kEndStr = kStr;
        return this;
    }

    public RobotConfig setVelocityGains(double fwd, double str) {
        velGainFwd = fwd;
        velGainStr = str;
        return this;
    }
}
