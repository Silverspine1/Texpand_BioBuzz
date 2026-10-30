package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Localisation;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.CommandBase.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.CommandBase.GoBildaPinpointDriver.Register;
import org.firstinspires.ftc.teamcode.CommandBase.LoopProfiler;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_command_base.Commands.LambdaCommand;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;

public class Odometry extends SubSystem {

    private final LambdaCommand resetPosition = new LambdaCommand(
            () -> {},
            () -> {

            },
            () -> false
    );

    // GoBilda Pinpoint
    public GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;

    Register[] onlyPosition = {
            Register.X_POSITION,
            Register.Y_POSITION,
            Register.H_ORIENTATION,
            Register.X_VELOCITY,
            Register.Y_VELOCITY,
    };

    public double X, Y, Heading, normilised;
    double startX, startY, startHeading;

    double XVelocity = 0;
    double YVelocity = 0;
    double HVelocity = 0;

    // Pinpoint acceleration and heading velocity
    // X/Y acceleration: cm/s²
    // Heading acceleration: rad/s²
    // Heading velocity: rad/s
    double XAcceleration = 0;
    double YAcceleration = 0;
    double HAcceleration = 0;
    double HeadingVelocity = 0;

    private long lastPinpointVelocitySampleNanos = 0;

    boolean resetAtStart = false;

    public Odometry(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, update);
    }

    public void startPosition(double X, double Y, int Heading) {
        this.startX = X;
        this.startY = Y;
        this.startHeading = Heading;
        this.Heading = normalizeDegrees(Heading);
        this.normilised = normalizeRadians(Math.toRadians(Heading));
    }

    @Override
    public void init() {
        odo = getOpMode().hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // --- Pinpoint I2C Optimization (Phase 2) ---
        // Only read what we actually use. Excluding LOOP_TIME, etc.
        odo.setBulkReadScope(
                Register.X_POSITION,
                Register.Y_POSITION,
                Register.H_ORIENTATION,
                Register.X_VELOCITY,
                Register.Y_VELOCITY,
                Register.H_VELOCITY);

        odo.setOffsets(45, 30, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
    }

    public double headingError(double targetHeading) {
        return Heading - targetHeading;
    }

    @Override
    public void execute() {
        long start = System.nanoTime();
        executeEX();
        ((OpModeEX) getOpMode()).profiler.recordDuration(LoopProfiler.ODOMETRY, System.nanoTime() - start);
    }

    public double X() {
        return X;
    }

    public double Y() {
        return Y;
    }

    public double Heading() {
        return normalizeDegrees(360 - Heading);
    }

    public double normiliased() {
        return normilised;
    }

    public double getYVelocity() {
        return YVelocity;
    }

    public double getXVelocity() {
        return XVelocity;
    }

    public double getHVelocity() {
        return HVelocity;
    }

    // Pinpoint acceleration getters

    public double getXAcceleration() {
        return XAcceleration;
    }

    public double getYAcceleration() {
        return YAcceleration;
    }

    public double getHAcceleration() {
        return HAcceleration;
    }

    public double getHeadingVelocity() {
        return HeadingVelocity;
    }

    /** Heading rate in deg/s, same direction convention as Heading(). */
    public double getHeadingVelocityDeg() {
        return -Math.toDegrees(HeadingVelocity);
    }

    /** Heading acceleration in deg/s^2, same direction convention as Heading(). */
    public double getHeadingAccelerationDeg() {
        return -Math.toDegrees(HAcceleration);
    }

    private boolean posePending = false;
    private double pendingX, pendingY, pendingRaw;

    /**
     * Sets the current pose (field cm, and heading in the same convention as Heading()).
     * The pose is written INTO the Pinpoint (once it reports READY - a write during
     * the reset at init would be lost), so it keeps tracking in the field frame.
     * The old way only offset heading in software: X/Y stayed in the Pinpoint's own
     * frame, rotated against Heading() by however far the robot had turned, so every
     * field-to-robot conversion (path following, vision ball map, aiming) was off by
     * that angle.
     */
    public void setPose(double xCm, double yCm, double headingDeg) {
        double rawTarget = normalizeDegrees(360 - headingDeg);
        pendingX = xCm;
        pendingY = yCm;
        pendingRaw = rawTarget;
        posePending = true;
        X = xCm;
        Y = yCm;
        Heading = rawTarget;
        normilised = normalizeRadians(Math.toRadians(rawTarget));
    }

    /** Writes a pending pose once the Pinpoint is READY. @return true if it was written this call. */
    private boolean applyPendingPose() {
        if (!posePending) return false;
        if (odo.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) return false;
        double raw = pendingRaw > 180 ? pendingRaw - 360 : pendingRaw;
        // inverse of the mapping in update(): X = startX - posX, Y = startY + posY, Heading = startHeading + raw
        odo.setPosition(new Pose2D(DistanceUnit.CM, -pendingX, pendingY, AngleUnit.DEGREES, raw));
        startX = 0;
        startY = 0;
        startHeading = 0;
        posePending = false;
        return true;
    }

    // Normalize a degree value into [0, 360)
    private double normalizeDegrees(double deg) {
        deg %= 360.0;
        if (deg < 0) deg += 360.0;
        return deg;
    }

    // Normalize a radian value into [0, 2*PI)
    private double normalizeRadians(double rad) {
        double twoPi = 2.0 * Math.PI;
        rad %= twoPi;
        if (rad < 0) rad += twoPi;
        return rad;
    }

    public LambdaCommand update = new LambdaCommand(
            () -> {
            },
            () -> {

                // pose written this loop: the bulk read below would still be the old pose
                if (applyPendingPose() || posePending) return;

                odo.update();

                // Store previous velocities for Pinpoint acceleration calculation
                double previousXVelocity = XVelocity;
                double previousYVelocity = YVelocity;
                double previousHVelocity = HVelocity;

                XVelocity = -odo.getVelX(DistanceUnit.CM);
                YVelocity = odo.getVelY(DistanceUnit.CM);
                HVelocity = odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

                HeadingVelocity = HVelocity;

                long currentTimeNanos = System.nanoTime();

                if (lastPinpointVelocitySampleNanos != 0) {

                    double deltaTime = (currentTimeNanos - lastPinpointVelocitySampleNanos) / 1_000_000_000.0;

                    if (deltaTime > 0) {
                        XAcceleration = (XVelocity - previousXVelocity) / deltaTime;
                        YAcceleration = (YVelocity - previousYVelocity) / deltaTime;
                        HAcceleration = (HVelocity - previousHVelocity) / deltaTime;
                    }
                }

                lastPinpointVelocitySampleNanos = currentTimeNanos;

                double rawDeg = odo.getHeading(AngleUnit.DEGREES);
                double rawRad = odo.getHeading(AngleUnit.RADIANS);

                Heading = normalizeDegrees(startHeading + rawDeg);
                normilised = normalizeRadians(Math.toRadians(startHeading) + rawRad);

                X = startX - odo.getPosX(DistanceUnit.CM);
                Y = startY + odo.getPosY(DistanceUnit.CM);
            },
            () -> false);


}