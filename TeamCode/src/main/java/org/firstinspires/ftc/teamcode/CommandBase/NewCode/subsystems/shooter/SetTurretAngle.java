package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.Servo;

public class SetTurretAngle {

    private static final double PROFILE_EPSILON = 0.000001;
    private static final double MAX_DT = 0.05;

    private final Servo tServ;
    private final Servo tServ1;

    private final AxonEncoder encoder;

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


    public SetTurretAngle(
            Servo servo,
            Servo turretServo2, AxonEncoder encoder,
            double maxVelocity,
            double maxAcceleration
    ) {
        this.tServ = servo;
        this.tServ1 = turretServo2;
        this.encoder = encoder;

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

        long now = System.nanoTime();

        double rawPosition = encoder.getTurretAngle();
        double rawVelocity = encoder.getVelocity();

        if (Double.isNaN(rawPosition) || Double.isInfinite(rawPosition)
                || Double.isNaN(rawVelocity) || Double.isInfinite(rawVelocity)) {
            tServ.setPosition(neutralPosition);
            tServ1.setPosition(neutralPosition);
            return;
        }

        Position = rawPosition;
        Velocity = rawVelocity;

        if (targetNeedsResolution) {
            targetPosition = resolveTarget(
                    requestedAngle,
                    Position,
                    useExtraRange
            );

            targetNeedsResolution = false;
        }

        if (needsProfileInitialization) {
            initializeProfile(now);
            calculateControl();
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
        calculateControl();
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
            return clamp(baseAngle, minPosition, maxPosition);
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


    private void initializeProfile(long now) {
        profilePosition = Position;
        profileVelocity = Velocity;
        profileAcceleration = 0;

        profileFinished = false;

        lastUpdateNanos = now;
        needsProfileInitialization = false;
    }


    private void updateProfile(double dt) {
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

        double stoppingVelocity =
                Math.sqrt(
                        2.0
                                * maxAcceleration
                                * Math.abs(distanceRemaining)
                );

        double requestedVelocity =
                direction
                        * Math.min(
                        maxVelocity,
                        stoppingVelocity
                );

        double oldVelocity = profileVelocity;

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

        if (movingSlowly
                && (crossedTarget || closeToTarget)) {

            profilePosition = targetPosition;
            profileVelocity = 0;
            profileAcceleration = 0;

            profileFinished = true;
            return;
        }

        profilePosition = nextPosition;
        profileFinished = false;
    }


    private void calculateControl() {
        positionError =
                profilePosition - Position;

        velocityError =
                profileVelocity - Velocity;

        double movementDirection;

        if (Math.abs(profileVelocity) > PROFILE_EPSILON) {
            movementDirection =
                    Math.signum(profileVelocity);
        } else {
            movementDirection =
                    Math.signum(profileAcceleration);
        }

        feedforward =
                kS * movementDirection
                        + kV * profileVelocity
                        + kA * profileAcceleration;

        feedback =
                kP * positionError
                        + kVelocityFeedback * velocityError;

        output =
                clamp(
                        feedforward + feedback,
                        -maxCorrection,
                        maxCorrection
                );

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

        tServ.setPosition(servoCommand);
        tServ1.setPosition(servoCommand);
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
        atTarget = false;
        profileFinished = false;

        insideToleranceSinceNanos = 0;

        tServ.setPosition(neutralPosition);
        tServ1.setPosition(neutralPosition);
    }


    public void setConstraints(
            double maxVelocity,
            double maxAcceleration
    ) {
        this.maxVelocity = Math.abs(maxVelocity);
        this.maxAcceleration = Math.abs(maxAcceleration);
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