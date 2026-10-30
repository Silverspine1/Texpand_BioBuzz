package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.Servo;

public class SetTurretAngle {

    private static final double PROFILE_EPSILON = 0.000001;
    private static final double MAX_DT = 0.05;

    /** Where the controller reads the turret from (the real encoder, or the tuner's plant model). */
    public interface Sensor {
        double getTurretAngle();

        double getVelocity();
    }

    /** Where the servo command goes (both servos on the robot). */
    public interface Sink {
        void set(double position);
    }

    private final Sensor encoder;
    private final Sink output_;

    private double minPosition = -140;
    private double maxPosition = 140;

    private double maxVelocity = 180;
    private double maxAcceleration = 1000;

    private double kP = 0.004;
    private double kVelocityFeedback = 0.0;

    private double kS = 0.0;
    private double kV = 0.0020;
    private double kA = 0.0;

    private double neutralPosition = 0.5;
    private double maxCorrection = 0.4;

    private double requestedAngle;
    private double targetPosition;

    private double positionTolerance = 1.0;
    private double velocityTolerance = 10.0;
    private double settleTime = 0.05;

    private double profilePosition;
    private double profileVelocity;
    private double profileAcceleration;

    private double Position;
    private double Velocity;

    private double positionError;
    private double velocityError;

    private double feedforward;
    private double feedback;
    private double output;
    private double servoCommand;

    private long lastUpdateNanos;
    private long insideToleranceSinceNanos;

    private boolean active = false;
    private boolean profileFinished = false;
    private boolean atTarget = false;
    private boolean needsProfileInitialization = false;

    private boolean targetNeedsResolution = false;
    private boolean useExtraRange = false;

    private double lastSafeServoCommand = 0.5;
    private double retargetThreshold = 0.5;

    private boolean usingSavedTune = false;

    // Feedback compares against where the profile was this long ago, so normal lag isn't treated as error.
    private double latencyCompensation = 0;

    private double[] ffSpeed;
    private double[] ffPower;


    // Brake-aware mode: the CR servo pair is a velocity source whose neutral is a hard brake, so the turret is driven
    // by a speed demand (stops in the remaining distance given the loop delay) instead of a position profile.
    // Demands under the creep speed are pulse-width modulated between creep and neutral so every pulse ends in a stop.
    // Final approach: below the breakaway power P alone cannot close the last degrees (it just stalls short), so while the
    // profile is done and the turret is nearly still a short pulse of 1.3 x kS is sent, then it waits to see where it stopped.
    private static final double NUDGE_GAIN = 1.3;
    private static final double NUDGE_PULSE = 0.015;
    private static final double NUDGE_REST = 0.12;
    private static final double NUDGE_MAX_SPEED = 8.0;
    private long pulseEndNanos = 0;
    private long pulseRestEndNanos = 0;
    private double pulseSign = 0;

    private boolean brakeAware = false;   // "predictive" mode; the name is kept so saved tunes still load
    private double plantDelay = 0.04;
    private double plantAccel = 1500;
    private double plantDecel = 1500;
    private double kPos = 15;
    private double decelSafety = 0.8;
    private static final int ISSUED_HISTORY = 96;
    private static final double OBSERVER_CORRECTION = 0.02;
    private final double[] issuedTime = new double[ISSUED_HISTORY];
    private final double[] issuedSpeed = new double[ISSUED_HISTORY];
    private int issuedHead = 0;
    private int issuedCount = 0;
    private double observerVelocity = 0;
    private double predictedPosition, predictedVelocity, speedDemand;
    private long observerLastNanos = 0;
    private double brakeDelayScale = 1.0;
    private static final double BRAKE_HOLD_DEG = 0.15;
    private static final double DITHER_PERIOD = 0.04;

    private static final int HISTORY_SIZE = 64;
    private final long[] historyTime = new long[HISTORY_SIZE];
    private final double[] historyPosition = new double[HISTORY_SIZE];
    private final double[] historyVelocity = new double[HISTORY_SIZE];
    private int historyHead = 0;
    private int historyCount = 0;
    private double delayedProfilePosition;
    private double delayedProfileVelocity;

    // A continuously moving target is led by targetRate * latencyCompensation so tracking doesn't lag.
    private static final double TRACKING_JUMP = 4.0;
    private static final double MAX_LEAD = 3.0;
    private static final double TRACKING_TIMEOUT = 0.25;
    private double targetRate = 0;
    private double targetAccel = 0;
    private double targetFiltered = 0;
    // critically damped g-h-k gains for theta = 0.72
    private static final double TRACK_ALPHA = 1 - 0.72 * 0.72 * 0.72;
    private static final double TRACK_BETA = 1.5 * (1 - 0.72) * (1 - 0.72) * (1 + 0.72);
    private static final double TRACK_GAMMA = 0.5 * (1 - 0.72) * (1 - 0.72) * (1 - 0.72);
    private static final double MAX_TARGET_ACCEL = 3000;
    private long lastRequestNanos = 0;

    // Behind the turret's travel limits the target sits at one limit; near the +/-180 seam it keeps the side it was on.
    private static final double DEAD_ZONE_HYSTERESIS = 15.0;
    private double lastResolvedSide = 0;


    public SetTurretAngle(
            Servo servo,
            Servo turretServo2, AxonEncoder encoder,
            double maxVelocity,
            double maxAcceleration
    ) {
        this.encoder = encoder;
        this.output_ = new Sink() {
            @Override
            public void set(double position) {
                servo.setPosition(position);
                turretServo2.setPosition(position);
            }
        };

        setConstraints(maxVelocity, maxAcceleration);

        TurretTune saved = TurretTune.load();
        if (saved != null) {
            saved.applyTo(this);
            usingSavedTune = true;
        }
    }


    /** Controller on a model (no saved tune is loaded): used by the tuner to score gains without moving the turret. */
    public SetTurretAngle(Sensor sensor, Sink sink, double maxVelocity, double maxAcceleration) {
        this.encoder = sensor;
        this.output_ = sink;
        setConstraints(maxVelocity, maxAcceleration);
    }


    public void setTarget(
            double angle,
            double tolerance,
            boolean useExtraRange
    ) {
        if (Double.isNaN(angle) || Double.isInfinite(angle)) {
            return;
        }

        boolean targetMoved = !active
                || Math.abs(normalizeAngle(angle - requestedAngle)) > retargetThreshold
                || useExtraRange != this.useExtraRange;

        long now = nowNanos();
        double sinceLast = (now - lastRequestNanos) / 1e9;
        double change = normalizeAngle(angle - requestedAngle);
        if (active && lastRequestNanos != 0 && sinceLast > 0
                && sinceLast < TRACKING_TIMEOUT && Math.abs(change) < TRACKING_JUMP) {
            // g-h-k filter on the requested angle: follows a smooth ride without the lag a plain low-pass puts in the rate.
            double predicted = targetFiltered + targetRate * sinceLast + 0.5 * targetAccel * sinceLast * sinceLast;
            double residual = normalizeAngle(angle - predicted);
            targetFiltered = predicted + TRACK_ALPHA * residual;
            targetRate += targetAccel * sinceLast + TRACK_BETA / sinceLast * residual;
            targetAccel += 2.0 * TRACK_GAMMA / (sinceLast * sinceLast) * residual;
            targetAccel = clamp(targetAccel, -MAX_TARGET_ACCEL, MAX_TARGET_ACCEL);
        } else if (lastRequestNanos == 0 || sinceLast > 0) {
            targetRate = 0;
            targetAccel = 0;
            targetFiltered = angle;
        }
        lastRequestNanos = now;

        requestedAngle = angle;
        positionTolerance = Math.abs(tolerance);

        this.useExtraRange = useExtraRange;
        targetNeedsResolution = true;

        if (targetMoved) {
            atTarget = false;
            insideToleranceSinceNanos = 0;
        }

        if (!active) {
            active = true;
            needsProfileInitialization = true;
            lastUpdateNanos = 0;
        }
    }


    public void setTarget(
            double angle,
            double tolerance
    ) {
        setTarget(angle, tolerance, false);
    }


    public void update() {
        if (!active) {
            return;
        }

        long now = nowNanos();

        double rawPosition = encoder.getTurretAngle();
        double rawVelocity = encoder.getVelocity();

        if (Double.isNaN(rawPosition) || Double.isInfinite(rawPosition)
                || Double.isNaN(rawVelocity) || Double.isInfinite(rawVelocity)) {
            output_.set(neutralPosition);
            return;
        }

        Position = rawPosition;
        Velocity = rawVelocity;

        if (targetRate != 0 && (now - lastRequestNanos) / 1e9 > TRACKING_TIMEOUT) {
            targetRate = 0;
            targetAccel = 0;
            targetNeedsResolution = true;
        }

        if (targetNeedsResolution) {
            targetPosition = resolveTarget(
                    requestedAngle + (brakeAware ? 0 : clamp(targetRate * latencyCompensation, -MAX_LEAD, MAX_LEAD)),
                    Position,
                    useExtraRange
            );

            targetNeedsResolution = false;
        }

        if (needsProfileInitialization) {
            initializeProfile(now);
            recordProfile(now);
            calculateControl(now);
            updateAtTarget(now);
            return;
        }

        double dt =
                (now - lastUpdateNanos)
                        / 1_000_000_000.0;

        lastUpdateNanos = now;

        if (dt <= 0) {
            return;
        }

        dt = Math.min(dt, MAX_DT);

        updateProfile(dt);
        recordProfile(now);
        calculateControl(now);
        updateAtTarget(now);
    }


    private double resolveTarget(
            double angle,
            double currentPosition,
            boolean useExtraRange
    ) {
        double baseAngle = normalizeAngle(angle);

        int minimumTurn =
                (int) Math.ceil(
                        (minPosition - baseAngle) / 360.0
                );

        int maximumTurn =
                (int) Math.floor(
                        (maxPosition - baseAngle) / 360.0
                );

        if (minimumTurn > maximumTurn) {
            double side = Math.signum(baseAngle);
            if (Math.abs(baseAngle) > 180.0 - DEAD_ZONE_HYSTERESIS && lastResolvedSide != 0) {
                side = lastResolvedSide;
            }
            lastResolvedSide = side;
            return side > 0 ? maxPosition : minPosition;
        }

        double bestTarget = baseAngle;
        double bestScore = Double.POSITIVE_INFINITY;

        for (int turn = minimumTurn; turn <= maximumTurn; turn++) {

            double possibleTarget =
                    baseAngle + turn * 360.0;

            double score;

            if (useExtraRange) {
                score =
                        Math.abs(
                                possibleTarget - currentPosition
                        );
            } else {
                score =
                        Math.abs(possibleTarget);
            }

            if (score < bestScore) {
                bestScore = score;
                bestTarget = possibleTarget;
            }
        }

        if (bestTarget != 0) {
            lastResolvedSide = Math.signum(bestTarget);
        }
        return bestTarget;
    }


    private double normalizeAngle(double angle) {
        angle %= 360.0;

        if (angle >= 180.0) {
            angle -= 360.0;
        }

        if (angle < -180.0) {
            angle += 360.0;
        }

        return angle;
    }


    private void recordProfile(long now) {
        historyHead = (historyHead + 1) % HISTORY_SIZE;
        historyTime[historyHead] = now;
        historyPosition[historyHead] = profilePosition;
        historyVelocity[historyHead] = profileVelocity;
        historyCount = Math.min(historyCount + 1, HISTORY_SIZE);
    }


    private void lookUpDelayedProfile(long time) {
        delayedProfilePosition = profilePosition;
        delayedProfileVelocity = profileVelocity;
        if (latencyCompensation <= 0 || historyCount == 0) {
            return;
        }

        int newer = historyHead;
        for (int i = 1; i < historyCount; i++) {
            int older = (historyHead - i + HISTORY_SIZE) % HISTORY_SIZE;
            if (historyTime[older] <= time) {
                long span = historyTime[newer] - historyTime[older];
                double f = span > 0 ? (double) (time - historyTime[older]) / span : 0;
                delayedProfilePosition = historyPosition[older] + f * (historyPosition[newer] - historyPosition[older]);
                delayedProfileVelocity = historyVelocity[older] + f * (historyVelocity[newer] - historyVelocity[older]);
                return;
            }
            newer = older;
        }
        delayedProfilePosition = historyPosition[newer];
        delayedProfileVelocity = historyVelocity[newer];
    }


    private void initializeProfile(long now) {
        historyCount = 0;
        profilePosition = Position;
        profileVelocity = Velocity;
        profileAcceleration = 0;

        observerVelocity = Velocity;
        observerLastNanos = 0;
        issuedCount = 0;

        profileFinished = false;

        lastUpdateNanos = now;
        needsProfileInitialization = false;
    }


    private void updateProfile(double dt) {
        if (brakeAware) {
            profilePosition = targetPosition;
            profileVelocity = 0;
            profileAcceleration = 0;
            profileFinished = true;
            return;
        }

        double distanceRemaining =
                targetPosition - profilePosition;

        if (Math.abs(distanceRemaining) < PROFILE_EPSILON
                && Math.abs(profileVelocity) < PROFILE_EPSILON) {

            profilePosition = targetPosition;
            profileVelocity = 0;
            profileAcceleration = 0;

            profileFinished = true;
            return;
        }

        double direction;

        if (Math.abs(distanceRemaining) > PROFILE_EPSILON) {
            direction = Math.signum(distanceRemaining);
        } else {
            direction = -Math.signum(profileVelocity);
        }

        // Fastest speed that can still stop at the target, measured from where this step will end
        // (not where it starts) so a whole loop of travel can't carry the profile past the target.
        double oldVelocity = profileVelocity;
        double a = maxAcceleration;
        double distanceAfterStep =
                Math.abs(distanceRemaining) - Math.max(0, oldVelocity * direction) * dt / 2.0;
        double stoppingVelocity =
                Math.max(0, (-a * dt + Math.sqrt(a * a * dt * dt + 8.0 * a * Math.max(0, distanceAfterStep))) / 2.0);

        double requestedVelocity =
                direction
                        * Math.min(
                        maxVelocity,
                        stoppingVelocity
                );

        profileVelocity =
                moveTowards(
                        profileVelocity,
                        requestedVelocity,
                        maxAcceleration * dt
                );

        profileAcceleration =
                (profileVelocity - oldVelocity) / dt;

        double nextPosition =
                profilePosition
                        + ((oldVelocity + profileVelocity) / 2.0)
                        * dt;

        double nextDistance =
                targetPosition - nextPosition;

        boolean crossedTarget =
                Math.signum(distanceRemaining) != 0
                        && Math.signum(nextDistance)
                        != Math.signum(distanceRemaining);

        boolean closeToTarget =
                Math.abs(nextDistance)
                        <= 0.5
                        * maxAcceleration
                        * dt
                        * dt;

        boolean movingSlowly =
                Math.abs(profileVelocity)
                        <= 1.25
                        * maxAcceleration
                        * dt;

        if (crossedTarget || (movingSlowly && closeToTarget)) {

            profilePosition = targetPosition;
            profileVelocity = 0;
            profileAcceleration = 0;

            profileFinished = true;
            return;
        }

        profilePosition = nextPosition;
        profileFinished = false;
    }


    private void computeProfiledOutput(long now) {
        lookUpDelayedProfile(now - (long) (latencyCompensation * 1e9));

        positionError =
                delayedProfilePosition - Position;

        velocityError =
                delayedProfileVelocity - Velocity;

        double movementDirection;

        if (Math.abs(profileVelocity) > PROFILE_EPSILON) {
            movementDirection =
                    Math.signum(profileVelocity);
        } else if (Math.abs(profileAcceleration) > PROFILE_EPSILON) {
            movementDirection =
                    Math.signum(profileAcceleration);
        } else if (Math.abs(positionError) > positionTolerance) {
            // Full kS only well outside tolerance: a full-size kS flipping sign with the error is a limit cycle
            // on a servo that lags the command by ~50 ms.
            movementDirection =
                    Math.signum(positionError)
                            * Math.min(1.0, (Math.abs(positionError) - positionTolerance) / (3.0 * positionTolerance));
        } else {
            movementDirection = 0;
        }


        if (ffSpeed != null && Math.abs(profileVelocity) > PROFILE_EPSILON) {
            feedforward =
                    Math.signum(profileVelocity) * tablePower(Math.abs(profileVelocity))
                            + kA * profileAcceleration;
        } else {
            feedforward =
                    kS * movementDirection
                            + kV * profileVelocity
                            + kA * profileAcceleration;
        }

        feedback =
                kP * positionError
                        + kVelocityFeedback * velocityError;

        output =
                clamp(
                        feedforward + feedback,
                        -maxCorrection,
                        maxCorrection
                );

        applyNudge(now);
    }


    private void applyNudge(long now) {
        double pulsePower = kS * NUDGE_GAIN;
        if (pulsePower <= 0 || Math.abs(profileVelocity) > PROFILE_EPSILON) {
            pulseEndNanos = 0;
            return;
        }
        if (now < pulseEndNanos) {
            output = pulseSign * pulsePower;
            return;
        }
        double error = targetPosition - Position;
        if (Math.abs(error) > 0.6 * positionTolerance
                && Math.abs(Velocity) < NUDGE_MAX_SPEED
                && now >= pulseRestEndNanos
                && Math.abs(output) < pulsePower) {
            pulseSign = Math.signum(error);
            pulseEndNanos = now + (long) (NUDGE_PULSE * 1e9);
            pulseRestEndNanos = pulseEndNanos + (long) (NUDGE_REST * 1e9);
            output = pulseSign * pulsePower;
        }
    }


    /** Speed demand -> power: the measured table when there is one, otherwise kS + kV * speed. */
    private double powerForSpeed(double speed) {
        if (ffSpeed != null) {
            return tablePower(speed);
        }
        return kS + kV * speed;
    }


    /** Slowest steady speed the servos can hold: what 1.3 x kS (just over breakaway) gives. */
    private double creepSpeed() {
        double power = 1.3 * kS;
        if (ffSpeed != null && ffSpeed.length > 1) {
            int last = ffPower.length - 1;
            if (power >= ffPower[last]) return ffSpeed[last];
            int i = 1;
            while (i < last && power > ffPower[i]) i++;
            double span = ffPower[i] - ffPower[i - 1];
            double f = span > 1e-9 ? Math.max(0, (power - ffPower[i - 1]) / span) : 1;
            return Math.max(3.0, ffSpeed[i - 1] + f * (ffSpeed[i] - ffSpeed[i - 1]));
        }
        return Math.max(5.0, (power - kS) / Math.max(1e-6, kV));
    }


    /**
     * Speed-demand control for a servo pair that is a speed source with a delay and a slew limit.
     * An observer runs the same delay + slew model on the speeds already commanded, so the state the new command will
     * meet (position and speed one delay from now) is known; the speed demand is the fastest that still stops on the
     * target from there, plus the target's own speed. Demands below the creep speed are pulse-width modulated.
     */
    private void computePredictiveOutput(long now) {
        double nowS = now / 1e9;
        double dt = observerLastNanos == 0 ? 0 : clamp((now - observerLastNanos) / 1e9, 0, 0.05);
        observerLastNanos = now;
        double delay = Math.max(0.005, plantDelay * brakeDelayScale);

        if (dt > 0) {
            observerVelocity = slew(observerVelocity, issuedSpeedAt(nowS - delay), dt);
            observerVelocity += OBSERVER_CORRECTION * (Velocity - observerVelocity);
        }

        double p = Position;
        double v = observerVelocity;
        double steps = Math.max(1, Math.ceil(delay / 0.005));
        double h = delay / steps;
        for (int i = 0; i < steps; i++) {
            v = slew(v, issuedSpeedAt(nowS + (i + 0.5) * h - delay), h);
            p += v * h;
        }

        double age = lastRequestNanos == 0 ? 0 : clamp((now - lastRequestNanos) / 1e9, 0, 0.1);
        double ahead = age + delay;
        double leadAccel = Math.abs(targetRate) > 1.0 ? targetAccel : 0;
        double error = targetPosition + targetRate * ahead + 0.5 * leadAccel * ahead * ahead - p;
        double absError = Math.abs(error);
        predictedPosition = p;
        predictedVelocity = v;

        positionError = targetPosition - Position;
        velocityError = 0;
        feedback = 0;

        double speed;
        if (absError < BRAKE_HOLD_DEG && Math.abs(targetRate) < 1.0) {
            speed = 0;
        } else {
            // Error left after the turret has matched the target's speed at the braking rate it is allowed: while the
            // target is still ahead of that point the demand is large, and it falls to zero exactly as braking must start.
            double rateNow = targetRate + leadAccel * ahead;
        double relativeSpeed = v - rateNow;
            double brakeRate = plantDecel * decelSafety;
            double remaining = error - relativeSpeed * Math.abs(relativeSpeed) / (2.0 * brakeRate);
            speed = clamp(rateNow + clamp(kPos * remaining, -maxVelocity, maxVelocity), -maxVelocity, maxVelocity);
        }

        double absSpeed = Math.abs(speed);
        double sign = Math.signum(speed);
        double creep = creepSpeed();
        double issued;

        if (absSpeed < 1e-9) {
            issued = 0;
            feedforward = 0;
            output = 0;
        } else {
            if (absSpeed < creep) {
                double duty = absSpeed / creep;
                double phase = (nowS % DITHER_PERIOD) / DITHER_PERIOD;
                absSpeed = phase < duty ? creep : 0;
            }
            if (absSpeed == 0) {
                issued = 0;
                feedforward = 0;
                output = 0;
            } else {
                double top = ffSpeed != null ? ffSpeed[ffSpeed.length - 1] : Double.MAX_VALUE;
                absSpeed = Math.min(absSpeed, top);
                feedforward = sign * clamp(powerForSpeed(absSpeed), 0, maxCorrection);
                output = feedforward;
                issued = sign * absSpeed;
            }
        }
        speedDemand = speed;
        recordIssuedSpeed(nowS, issued);
    }

    private double slew(double current, double target, double dt) {
        double rate = Math.abs(target) >= Math.abs(current) && target * current >= 0 ? plantAccel : plantDecel;
        if (Math.abs(target) < 1e-9 || target * current < 0) {
            rate = plantDecel;
        }
        return moveTowards(current, target, rate * dt);
    }

    private void recordIssuedSpeed(double timeS, double speed) {
        issuedHead = (issuedHead + 1) % ISSUED_HISTORY;
        issuedTime[issuedHead] = timeS;
        issuedSpeed[issuedHead] = speed;
        issuedCount = Math.min(issuedCount + 1, ISSUED_HISTORY);
    }

    /** The speed demand that was in force at a time (the latest one issued at or before it). */
    private double issuedSpeedAt(double timeS) {
        for (int i = 0; i < issuedCount; i++) {
            int idx = (issuedHead - i + ISSUED_HISTORY) % ISSUED_HISTORY;
            if (issuedTime[idx] <= timeS) {
                return issuedSpeed[idx];
            }
        }
        return 0;
    }


    private void calculateControl(long now) {
        if (brakeAware) {
            computePredictiveOutput(now);
        } else {
            computeProfiledOutput(now);
        }


        if (Position <= minPosition && output < 0) {
            output = 0;
        }

        if (Position >= maxPosition && output > 0) {
            output = 0;
        }

        servoCommand =
                clamp(
                        neutralPosition + output,
                        0,
                        1
                );

        if (Double.isNaN(servoCommand) || Double.isInfinite(servoCommand)) {
            servoCommand = lastSafeServoCommand;
        } else {
            lastSafeServoCommand = servoCommand;
        }

        output_.set(servoCommand);
    }


    private void updateAtTarget(long now) {
        boolean insideTolerance =
                Math.abs(targetPosition - Position)
                        <= positionTolerance
                        && Math.abs(Velocity)
                        <= velocityTolerance;

        if (!insideTolerance) {
            insideToleranceSinceNanos = 0;
            atTarget = false;
            return;
        }

        if (insideToleranceSinceNanos == 0) {
            insideToleranceSinceNanos = now;
        }

        double timeInsideTolerance =
                (now - insideToleranceSinceNanos)
                        / 1_000_000_000.0;

        atTarget =
                timeInsideTolerance >= settleTime;
    }


    public void stop() {
        active = false;
        targetRate = 0;
        lastRequestNanos = 0;
        targetAccel = 0;
        atTarget = false;
        profileFinished = false;

        insideToleranceSinceNanos = 0;

        output_.set(neutralPosition);
    }


    public void setConstraints(
            double maxVelocity,
            double maxAcceleration
    ) {
        this.maxVelocity = Math.abs(maxVelocity);
        this.maxAcceleration = Math.abs(maxAcceleration);
    }


    public void setGains(
            double kS,
            double kV,
            double kA,
            double kP,
            double kVelocityFeedback,
            double maxCorrection
    ) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kP = kP;
        this.kVelocityFeedback = kVelocityFeedback;
        this.maxCorrection = Math.abs(maxCorrection);
    }


    public void setBrakeAware(boolean enabled, double delayScale) {
        brakeDelayScale = Math.max(0.5, Math.min(4.0, delayScale));
        if (enabled != brakeAware) {
            brakeAware = enabled;
            needsProfileInitialization = active;
            targetNeedsResolution = true;
        }
    }


    /** Delay (s), speed slew up / back toward neutral (deg/s^2), position gain (1/s) and stopping margin for predictive mode. */
    public void setPlant(double delay, double accel, double decel, double positionGain, double stopMargin) {
        plantDelay = Math.max(0.005, delay);
        plantAccel = Math.max(50, accel);
        plantDecel = Math.max(50, decel);
        kPos = Math.max(0.5, positionGain);
        decelSafety = Math.max(0.2, Math.min(1.0, stopMargin));
    }


    public boolean isBrakeAware() {
        return brakeAware;
    }



    public void setLatencyCompensation(double seconds) {
        latencyCompensation = Math.max(0, Math.min(0.5, seconds));
    }


    /** Measured speed -> power curve used for feedforward instead of kS + kV * speed. Null to disable. */
    public void setFeedforwardTable(double[] speed, double[] power) {
        if (TurretTune.isValidTable(speed, power) && speed != null) {
            ffSpeed = speed.clone();
            ffPower = power.clone();
        } else {
            ffSpeed = null;
            ffPower = null;
        }
    }


    private double tablePower(double speed) {
        int last = ffSpeed.length - 1;
        int i = 1;
        while (i < last && speed > ffSpeed[i]) i++;
        double span = ffSpeed[i] - ffSpeed[i - 1];
        double f = (speed - ffSpeed[i - 1]) / span;
        return ffPower[i - 1] + f * (ffPower[i] - ffPower[i - 1]);
    }


    public boolean isUsingSavedTune() {
        return usingSavedTune;
    }


    protected long nowNanos() {
        return System.nanoTime();
    }


    public boolean atTarget() {
        return atTarget;
    }


    public boolean isProfileFinished() {
        return profileFinished;
    }


    public boolean isActive() {
        return active;
    }


    public double getTargetPosition() {
        return targetPosition;
    }


    public double getActualPosition() {
        return Position;
    }


    public double getVelocity() {
        return Velocity;
    }


    public double getPositionError() {
        return positionError;
    }


    /** Predictive mode: where the observer expects the turret to be (deg), its speed (deg/s) and the speed demanded (deg/s). */
    public double getPredictedPosition() { return predictedPosition; }

    public double getPredictedVelocity() { return predictedVelocity; }

    public double getSpeedDemand() { return speedDemand; }

    public double getTargetRate() { return targetRate; }


    public double getServoCommand() {
        return servoCommand;
    }

    public double getOutput() {
        return output;
    }

    public double getProfilePosition() {
        return profilePosition;
    }

    public double getProfileVelocity() {
        return profileVelocity;
    }


    private double moveTowards(
            double current,
            double target,
            double maximumChange
    ) {
        double difference =
                target - current;

        if (Math.abs(difference) <= maximumChange) {
            return target;
        }

        return current
                + Math.signum(difference)
                * maximumChange;
    }

    private double clamp(
            double value,
            double minimum,
            double maximum
    ) {
        return Math.max(
                minimum,
                Math.min(maximum, value)
        );
    }
}