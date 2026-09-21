package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;


public class Shotplanner {


    public static class Config {
        public double pollenDiameterMeters = 0.072;
        public double flywheelRadiusMeters = 0.072;

        public double turretAxisForwardMeters = 0.1;
        public double turretAxisLeftMeters = 0.0;
        public double muzzleForwardFromTurretMeters = 0.05;
        public double muzzleLeftFromTurretMeters = 0.0;
        public double muzzleHeightMeters = 0.2;

        public double minLaunchAngleDeg = 25.0;
        public double maxLaunchAngleDeg = 80.0;
        public double maxFlywheelRPM = 6000.0;

        public double magnusK = 0.0195;
        public double requiredClearanceMeters = 0.025;

        public double cellWidthMeters = 0.508;
        public double cellVerticalSideHeightMeters = 0.1933;
        public double cellTotalHeightMeters = 0.3556;
        public double entryCheckDepthMeters = 0.10;
    }



    public static class RobotState {
        public final double x, y;
        public final double velocityX, velocityY;
        public final double accelerationX, accelerationY;
        public final double headingDeg;
        public final double angularVelocityDegPerSec;
        public final double angularAccelerationDegPerSec2;
        public final double turretAngularVelocityDegPerSec;

        public RobotState(double x, double y,
                          double velocityX, double velocityY,
                          double accelerationX, double accelerationY,
                          double headingDeg,
                          double angularVelocityDegPerSec,
                          double angularAccelerationDegPerSec2,
                          double turretAngularVelocityDegPerSec) {
            this.x = x;
            this.y = y;
            this.velocityX = velocityX;
            this.velocityY = velocityY;
            this.accelerationX = accelerationX;
            this.accelerationY = accelerationY;
            this.headingDeg = headingDeg;
            this.angularVelocityDegPerSec = angularVelocityDegPerSec;
            this.angularAccelerationDegPerSec2 = angularAccelerationDegPerSec2;
            this.turretAngularVelocityDegPerSec = turretAngularVelocityDegPerSec;
        }

        public RobotState(double x, double y,
                          double velocityX, double velocityY,
                          double headingDeg,
                          double angularVelocityDegPerSec,
                          double turretAngularVelocityDegPerSec) {
            this(
                    x, y,
                    velocityX, velocityY,
                    0, 0,
                    headingDeg,
                    angularVelocityDegPerSec,
                    0,
                    turretAngularVelocityDegPerSec
            );
        }
    }

    public static class HiveTarget {
        public final double openingX, openingY;
        public final double openingBottomZ;
        public final double facingAngleDeg;

        public HiveTarget(double openingX, double openingY,
                          double openingBottomZ, double facingAngleDeg) {
            this.openingX = openingX;
            this.openingY = openingY;
            this.openingBottomZ = openingBottomZ;
            this.facingAngleDeg = facingAngleDeg;
        }
    }
    public enum BlueHiveSide {
        AUDIENCE,
        OPPOSITE
    }

    public static HiveTarget getBlueHiveTarget(BlueHiveSide side) {
        switch (side) {
            case AUDIENCE:
                return new HiveTarget(
                        1.22,   // opening X: audience-side CELL, metres
                        2.15,   // opening Y: blue HIVE, metres
                        1.36,   // opening bottom height, metres
                        180.0   // opening faces toward audience
                );

            case OPPOSITE:
                return new HiveTarget(
                        2.44,   // opening X: opposite-side CELL, metres
                        2.15,   // opening Y: blue HIVE, metres
                        1.36,   // opening bottom height, metres
                        0.0     // opening faces away from audience
                );

            default:
                throw new IllegalArgumentException(
                        "Unknown blue HIVE side: " + side
                );
        }
    }

    public static class ShotSolution {
        public boolean reachable;
        public boolean safe;
        public double turretAngleDeg;
        public double launchAngleDeg;
        public double flywheelRPM;
        public double muzzleSpeedMetersPerSecond;
        public double flightTimeSeconds;
        public double minimumClearanceMeters;
    }

    public static class ShotEvaluation {
        public boolean entersCell;
        public boolean safe;
        public double minimumClearanceMeters;
    }

    private static final double GRAVITY = 9.81;
    private static final double DT = 0.01;
    private static final double MAX_FLIGHT_TIME = 2.0;
    private static final double MIN_MUZZLE_SPEED = 0.5;
    private static final double COARSE_ANGLE_STEP = 2.0;
    private static final double FINE_ANGLE_STEP = 0.25;
    private static final double FINE_ANGLE_RANGE = 2.0;
    private static final int SPEED_SEARCH_ITERATIONS = 16;
    private static final int YAW_CORRECTION_ITERATIONS = 6;
    private static final int CLEARANCE_SUBSTEPS = 5;

    Config config;

    public Shotplanner(Config config) {
        this.config = config;
        validateConfig();
    }

    public ShotSolution calculateShot(RobotState currentState,
                                      HiveTarget target,
                                      double mechanicalDelaySeconds) {
        RobotState releaseState = projectState(currentState, mechanicalDelaySeconds);
        return calculateShotAtRelease(releaseState, target);
    }

    public ShotSolution calculateShotAtRelease(RobotState releaseState,
                                               HiveTarget target) {
        Candidate best = findBestCandidate(releaseState, target);
        ShotSolution result = new ShotSolution();

        if (best == null) {
            result.minimumClearanceMeters = Double.NEGATIVE_INFINITY;
            return result;
        }

        result.reachable = true;
        result.safe = best.clearance >= config.requiredClearanceMeters;
        result.turretAngleDeg = normalizeDegrees(best.fieldYawDeg - releaseState.headingDeg);
        result.launchAngleDeg = best.launchAngleDeg;
        result.flywheelRPM = best.rpm;
        result.muzzleSpeedMetersPerSecond = best.muzzleSpeed;
        result.flightTimeSeconds = best.flightTime;
        result.minimumClearanceMeters = best.clearance;
        return result;
    }

    public ShotEvaluation evaluateShot(RobotState currentState,
                                       HiveTarget target,
                                       double mechanicalDelaySeconds,
                                       double turretAngleDeg,
                                       double launchAngleDeg,
                                       double flywheelRPM) {
        RobotState releaseState = projectState(currentState, mechanicalDelaySeconds);
        return evaluateShotAtRelease(releaseState, target, turretAngleDeg, launchAngleDeg, flywheelRPM);
    }

    public ShotEvaluation evaluateShotAtRelease(RobotState releaseState,
                                                HiveTarget target,
                                                double turretAngleDeg,
                                                double launchAngleDeg,
                                                double flywheelRPM) {
        double fieldYawDeg = normalizeDegrees(releaseState.headingDeg + turretAngleDeg);
        double muzzleSpeed = rpmToMuzzleSpeed(flywheelRPM);

        TrajectoryResult trajectory = simulate(
                releaseState, target, fieldYawDeg, launchAngleDeg, muzzleSpeed, true
        );

        ShotEvaluation result = new ShotEvaluation();
        result.entersCell = trajectory.enteredCorridor;
        result.minimumClearanceMeters = trajectory.minimumClearance;
        result.safe = trajectory.enteredCorridor
                && trajectory.minimumClearance >= config.requiredClearanceMeters;
        return result;
    }

    private Candidate findBestCandidate(RobotState robot, HiveTarget target) {
        Candidate best = null;

        for (double angle = config.minLaunchAngleDeg;
             angle <= config.maxLaunchAngleDeg + 1e-6;
             angle += COARSE_ANGLE_STEP) {
            best = chooseBetter(best, solveAngle(robot, target, angle));
        }

        if (best == null) return null;

        double start = Math.max(config.minLaunchAngleDeg, best.launchAngleDeg - FINE_ANGLE_RANGE);
        double end = Math.min(config.maxLaunchAngleDeg, best.launchAngleDeg + FINE_ANGLE_RANGE);

        for (double angle = start; angle <= end + 1e-6; angle += FINE_ANGLE_STEP) {
            best = chooseBetter(best, solveAngle(robot, target, angle));
        }

        return best;
    }

    private Candidate solveAngle(RobotState robot, HiveTarget target, double launchAngleDeg) {
        double fieldYawDeg = initialYaw(robot, target);

        for (int i = 0; i < YAW_CORRECTION_ITERATIONS; i++) {
            double speed = solveSpeed(robot, target, fieldYawDeg, launchAngleDeg);
            if (Double.isNaN(speed)) return null;

            TrajectoryResult trajectory = simulate(
                    robot, target, fieldYawDeg, launchAngleDeg, speed, false
            );
            if (!trajectory.crossedOpening) return null;

            MuzzleState muzzle = getMuzzleState(robot, fieldYawDeg);
            double targetHeading = Math.atan2(
                    target.openingY - muzzle.position.y,
                    target.openingX - muzzle.position.x
            );
            double crossingHeading = Math.atan2(
                    trajectory.frontCrossing.y - muzzle.position.y,
                    trajectory.frontCrossing.x - muzzle.position.x
            );

            double correction = normalizeRadians(targetHeading - crossingHeading);
            fieldYawDeg = normalizeDegrees(fieldYawDeg + Math.toDegrees(correction));
        }

        double speed = solveSpeed(robot, target, fieldYawDeg, launchAngleDeg);
        if (Double.isNaN(speed)) return null;

        double rpm = muzzleSpeedToRPM(speed);
        if (rpm > config.maxFlywheelRPM) return null;

        TrajectoryResult trajectory = simulate(
                robot, target, fieldYawDeg, launchAngleDeg, speed, true
        );
        if (!trajectory.crossedOpening) return null;

        Candidate candidate = new Candidate();
        candidate.fieldYawDeg = fieldYawDeg;
        candidate.launchAngleDeg = launchAngleDeg;
        candidate.muzzleSpeed = speed;
        candidate.rpm = rpm;
        candidate.clearance = trajectory.minimumClearance;
        candidate.flightTime = trajectory.flightTime;
        return candidate;
    }

    private double solveSpeed(RobotState robot, HiveTarget target,
                              double fieldYawDeg, double launchAngleDeg) {
        double low = MIN_MUZZLE_SPEED;
        double high = getMaxMuzzleSpeed();
        double targetZ = target.openingBottomZ + preferredAimHeight();

        TrajectoryResult highResult = simulate(
                robot, target, fieldYawDeg, launchAngleDeg, high, false
        );
        if (!highResult.crossedOpening || highResult.frontCrossing.z < targetZ) {
            return Double.NaN;
        }

        for (int i = 0; i < SPEED_SEARCH_ITERATIONS; i++) {
            double middle = (low + high) / 2.0;
            TrajectoryResult result = simulate(
                    robot, target, fieldYawDeg, launchAngleDeg, middle, false
            );

            if (!result.crossedOpening || result.frontCrossing.z < targetZ) low = middle;
            else high = middle;
        }

        return (low + high) / 2.0;
    }

    private TrajectoryResult simulate(RobotState robot,
                                      HiveTarget target,
                                      double fieldYawDeg,
                                      double launchAngleDeg,
                                      double muzzleSpeed,
                                      boolean calculateClearance) {
        MuzzleState muzzle = getMuzzleState(robot, fieldYawDeg);

        double yaw = Math.toRadians(fieldYawDeg);
        double elevation = Math.toRadians(launchAngleDeg);
        double horizontalSpeed = muzzleSpeed * Math.cos(elevation);

        Vec3 velocity = new Vec3(
                horizontalSpeed * Math.cos(yaw),
                horizontalSpeed * Math.sin(yaw),
                muzzleSpeed * Math.sin(elevation)
        ).add(muzzle.platformVelocity);

        Vec3 position = muzzle.position;
        Vec3 spinAxis = new Vec3(Math.sin(yaw), -Math.cos(yaw), 0);
        Vec3 outwardNormal = outwardNormal(target);

        double previousPlaneDistance = planeDistance(position, target, outwardNormal);
        double time = 0;

        TrajectoryResult result = new TrajectoryResult();
        result.minimumClearance = Double.POSITIVE_INFINITY;

        while (time < MAX_FLIGHT_TIME) {
            Vec3 acceleration = acceleration(velocity, spinAxis);
            Vec3 midVelocity = velocity.add(acceleration.multiply(DT / 2.0));
            Vec3 midAcceleration = acceleration(midVelocity, spinAxis);

            Vec3 nextPosition = position.add(midVelocity.multiply(DT));
            Vec3 nextVelocity = velocity.add(midAcceleration.multiply(DT));
            double nextPlaneDistance = planeDistance(nextPosition, target, outwardNormal);

            if (!result.crossedOpening
                    && previousPlaneDistance > 0
                    && nextPlaneDistance <= 0) {
                double fraction = previousPlaneDistance
                        / (previousPlaneDistance - nextPlaneDistance);

                result.crossedOpening = true;
                result.frontCrossing = interpolate(position, nextPosition, fraction);
                result.flightTime = time + fraction * DT;

                if (calculateClearance) {
                    result.minimumClearance = Math.min(
                            result.minimumClearance,
                            openingClearance(result.frontCrossing, target)
                    );
                }
            }

            if (calculateClearance && result.crossedOpening) {
                sampleClearance(position, nextPosition, target, result);

                if (entryDepth(nextPosition, target) >= config.entryCheckDepthMeters) {
                    result.enteredCorridor = true;
                    return result;
                }
            }

            if (nextPosition.z < 0) return result;

            position = nextPosition;
            velocity = nextVelocity;
            previousPlaneDistance = nextPlaneDistance;
            time += DT;
        }

        return result;
    }

    private Vec3 acceleration(Vec3 velocity, Vec3 spinAxis) {
        double speed = velocity.magnitude();
        Vec3 magnus = spinAxis.cross(velocity).multiply(config.magnusK * speed);
        return new Vec3(magnus.x, magnus.y, magnus.z - GRAVITY);
    }

    private void sampleClearance(Vec3 start, Vec3 end,
                                 HiveTarget target, TrajectoryResult result) {
        for (int i = 0; i <= CLEARANCE_SUBSTEPS; i++) {
            double t = (double) i / CLEARANCE_SUBSTEPS;
            Vec3 point = interpolate(start, end, t);
            double depth = entryDepth(point, target);

            if (depth >= 0 && depth <= config.entryCheckDepthMeters) {
                result.minimumClearance = Math.min(
                        result.minimumClearance,
                        openingClearance(point, target)
                );
            }
        }
    }

    private double openingClearance(Vec3 point, HiveTarget target) {
        double x = localRight(point, target);
        double y = point.z - target.openingBottomZ;
        double halfWidth = config.cellWidthMeters / 2.0;

        double[][] polygon = {
                {-halfWidth, 0},
                { halfWidth, 0},
                { halfWidth, config.cellVerticalSideHeightMeters},
                { 0, config.cellTotalHeightMeters},
                {-halfWidth, config.cellVerticalSideHeightMeters}
        };

        boolean inside = true;
        double minDistance = Double.POSITIVE_INFINITY;

        for (int i = 0; i < polygon.length; i++) {
            double[] a = polygon[i];
            double[] b = polygon[(i + 1) % polygon.length];
            double edgeX = b[0] - a[0];
            double edgeY = b[1] - a[1];
            double pointX = x - a[0];
            double pointY = y - a[1];

            if (edgeX * pointY - edgeY * pointX < 0) inside = false;

            minDistance = Math.min(
                    minDistance,
                    pointToSegmentDistance(x, y, a[0], a[1], b[0], b[1])
            );
        }

        double centerClearance = inside ? minDistance : -minDistance;
        return centerClearance - config.pollenDiameterMeters / 2.0;
    }

    private MuzzleState getMuzzleState(RobotState robot, double fieldYawDeg) {
        double robotHeading = Math.toRadians(robot.headingDeg);
        double fieldYaw = Math.toRadians(fieldYawDeg);

        Vec3 turretAxisOffset = rotate2D(
                config.turretAxisForwardMeters,
                config.turretAxisLeftMeters,
                robotHeading
        );
        Vec3 muzzleFromTurret = rotate2D(
                config.muzzleForwardFromTurretMeters,
                config.muzzleLeftFromTurretMeters,
                fieldYaw
        );

        Vec3 turretAxis = new Vec3(
                robot.x + turretAxisOffset.x,
                robot.y + turretAxisOffset.y,
                config.muzzleHeightMeters
        );
        Vec3 muzzlePosition = turretAxis.add(muzzleFromTurret);

        Vec3 robotToMuzzle = new Vec3(
                muzzlePosition.x - robot.x,
                muzzlePosition.y - robot.y,
                0
        );

        double robotOmega = Math.toRadians(robot.angularVelocityDegPerSec);
        double turretOmega = Math.toRadians(robot.turretAngularVelocityDegPerSec);

        Vec3 robotRotationVelocity = new Vec3(
                -robotOmega * robotToMuzzle.y,
                robotOmega * robotToMuzzle.x,
                0
        );
        Vec3 turretRotationVelocity = new Vec3(
                -turretOmega * muzzleFromTurret.y,
                turretOmega * muzzleFromTurret.x,
                0
        );

        MuzzleState result = new MuzzleState();
        result.position = muzzlePosition;
        result.platformVelocity = new Vec3(robot.velocityX, robot.velocityY, 0)
                .add(robotRotationVelocity)
                .add(turretRotationVelocity);
        return result;
    }

    private RobotState projectState(RobotState state, double delaySeconds) {
        double delay = Math.max(0, delaySeconds);
        double delaySquared = delay * delay;

        double releaseX =
                state.x
                        + state.velocityX * delay
                        + 0.5 * state.accelerationX * delaySquared;

        double releaseY =
                state.y
                        + state.velocityY * delay
                        + 0.5 * state.accelerationY * delaySquared;

        double releaseVelocityX =
                state.velocityX
                        + state.accelerationX * delay;

        double releaseVelocityY =
                state.velocityY
                        + state.accelerationY * delay;

        double releaseHeading =
                state.headingDeg
                        + state.angularVelocityDegPerSec * delay
                        + 0.5 * state.angularAccelerationDegPerSec2 * delaySquared;

        double releaseAngularVelocity =
                state.angularVelocityDegPerSec
                        + state.angularAccelerationDegPerSec2 * delay;

        return new RobotState(
                releaseX,
                releaseY,
                releaseVelocityX,
                releaseVelocityY,
                state.accelerationX,
                state.accelerationY,
                releaseHeading,
                releaseAngularVelocity,
                state.angularAccelerationDegPerSec2,
                state.turretAngularVelocityDegPerSec
        );
    }

    private double initialYaw(RobotState robot, HiveTarget target) {
        double heading = Math.toRadians(robot.headingDeg);
        Vec3 axisOffset = rotate2D(
                config.turretAxisForwardMeters,
                config.turretAxisLeftMeters,
                heading
        );
        return Math.toDegrees(Math.atan2(
                target.openingY - (robot.y + axisOffset.y),
                target.openingX - (robot.x + axisOffset.x)
        ));
    }

    private Candidate chooseBetter(Candidate current, Candidate next) {
        if (next == null) return current;
        if (current == null) return next;

        boolean currentSafe = current.clearance >= config.requiredClearanceMeters;
        boolean nextSafe = next.clearance >= config.requiredClearanceMeters;

        if (nextSafe != currentSafe) return nextSafe ? next : current;
        if (next.clearance > current.clearance + 0.001) return next;
        if (Math.abs(next.clearance - current.clearance) <= 0.001 && next.rpm < current.rpm) return next;
        return current;
    }

    private double preferredAimHeight() {
        double halfWidth = config.cellWidthMeters / 2.0;
        double roofRise = config.cellTotalHeightMeters - config.cellVerticalSideHeightMeters;
        double roofSlope = roofRise / halfWidth;
        double roofScale = Math.sqrt(1.0 + roofSlope * roofSlope);
        return config.cellTotalHeightMeters / (1.0 + roofScale);
    }

    private Vec3 outwardNormal(HiveTarget target) {
        double angle = Math.toRadians(target.facingAngleDeg);
        return new Vec3(Math.cos(angle), Math.sin(angle), 0);
    }

    private double planeDistance(Vec3 point, HiveTarget target, Vec3 normal) {
        return (point.x - target.openingX) * normal.x
                + (point.y - target.openingY) * normal.y;
    }

    private double entryDepth(Vec3 point, HiveTarget target) {
        return -planeDistance(point, target, outwardNormal(target));
    }

    private double localRight(Vec3 point, HiveTarget target) {
        double face = Math.toRadians(target.facingAngleDeg);
        double rightX = Math.sin(face);
        double rightY = -Math.cos(face);
        return (point.x - target.openingX) * rightX
                + (point.y - target.openingY) * rightY;
    }

    private double muzzleSpeedToRPM(double muzzleSpeed) {
        return 60.0 * muzzleSpeed / (Math.PI * config.flywheelRadiusMeters);
    }

    private double rpmToMuzzleSpeed(double rpm) {
        return Math.PI * config.flywheelRadiusMeters * rpm / 60.0;
    }

    private double getMaxMuzzleSpeed() {
        return rpmToMuzzleSpeed(config.maxFlywheelRPM);
    }

    private Vec3 rotate2D(double forward, double left, double angle) {
        return new Vec3(
                forward * Math.cos(angle) - left * Math.sin(angle),
                forward * Math.sin(angle) + left * Math.cos(angle),
                0
        );
    }

    private Vec3 interpolate(Vec3 start, Vec3 end, double t) {
        return new Vec3(
                start.x + (end.x - start.x) * t,
                start.y + (end.y - start.y) * t,
                start.z + (end.z - start.z) * t
        );
    }

    private double pointToSegmentDistance(double px, double py,
                                          double ax, double ay,
                                          double bx, double by) {
        double dx = bx - ax;
        double dy = by - ay;
        double lengthSquared = dx * dx + dy * dy;

        if (lengthSquared == 0) return Math.hypot(px - ax, py - ay);

        double t = ((px - ax) * dx + (py - ay) * dy) / lengthSquared;
        t = Math.max(0, Math.min(1, t));

        return Math.hypot(
                px - (ax + t * dx),
                py - (ay + t * dy)
        );
    }

    private double normalizeDegrees(double angle) {
        angle %= 360.0;
        if (angle > 180) angle -= 360;
        if (angle <= -180) angle += 360;
        return angle;
    }

    private double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    private void validateConfig() {
        if (config.flywheelRadiusMeters <= 0) throw new IllegalArgumentException("flywheelRadiusMeters must be > 0");
        if (config.pollenDiameterMeters <= 0) throw new IllegalArgumentException("pollenDiameterMeters must be > 0");
        if (config.maxFlywheelRPM <= 0) throw new IllegalArgumentException("maxFlywheelRPM must be > 0");
        if (config.maxLaunchAngleDeg <= config.minLaunchAngleDeg) throw new IllegalArgumentException("Launch angle limits are invalid");
        if (config.cellTotalHeightMeters <= config.cellVerticalSideHeightMeters) throw new IllegalArgumentException("CELL dimensions are invalid");
    }

    private static class Vec3 {
        double x, y, z;

        Vec3(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        Vec3 add(Vec3 other) {
            return new Vec3(x + other.x, y + other.y, z + other.z);
        }

        Vec3 multiply(double scalar) {
            return new Vec3(x * scalar, y * scalar, z * scalar);
        }

        Vec3 cross(Vec3 other) {
            return new Vec3(
                    y * other.z - z * other.y,
                    z * other.x - x * other.z,
                    x * other.y - y * other.x
            );
        }

        double magnitude() {
            return Math.sqrt(x * x + y * y + z * z);
        }
    }

    private static class MuzzleState {
        Vec3 position;
        Vec3 platformVelocity;
    }

    private static class TrajectoryResult {
        boolean crossedOpening;
        boolean enteredCorridor;
        Vec3 frontCrossing;
        double minimumClearance;
        double flightTime;
    }

    private static class Candidate {
        double fieldYawDeg;
        double launchAngleDeg;
        double muzzleSpeed;
        double rpm;
        double clearance;
        double flightTime;
    }
}
