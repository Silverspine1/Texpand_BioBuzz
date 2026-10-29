package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.Follower;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathingUtility.PathingVelocity;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.RobotUtilities.RobotConfig;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.RobotUtilities.Vector2D;

public class FollowerBaseMethods {

    PathOperator pathoperator;
    RobotConfig config;

    // heading
    boolean robotHeadingBackwards = false;
    double offset = 0;
    double headingOffset = 0;
    int headingLookAheadDistance = 20;
    int lookAheadIndex = 0;

    // toggles
    boolean disableGlobalFollowing = false;
    boolean slowerHeading = false;
    boolean forceStop = false;
    boolean useHeadingFrictionError = false;
    boolean holdPosition = false;
    boolean usePathHeadings = false;

    boolean pathFinished = false;
    boolean heldXY = false;          // inside holdT and holding still

    // robot data
    double velocityX = 0;
    double velocityY = 0;
    Vector2D robotPositionVector = new Vector2D();
    PathingVelocity targetVelocity = new PathingVelocity();
    Vector2D forceStopPoint = null;

    int currentIndex = 0;

    public void resetClosestPoint(Vector2D robotPos) {
        pathoperator.getRobotPositionOnPathFullPath(robotPos);
        pathoperator.resetClosest(robotPos.getX(), robotPos.getY());
    }

    public void createNewPathOperator(PathBuilder path) {
        pathoperator = new PathOperator(path.followablePath, path.pathingVelocity, path.headingTargets,
                config.latAccel, Math.min(config.MAX_X_ACCELERATION(), config.MAX_Y_ACCELERATION()));
    }

    public int clipLookAheadIndex(int index) {
        if (index > pathoperator.targetHeadings.size() - 1) {
            index = pathoperator.targetHeadings.size() - 1;
        }
        lookAheadIndex = Math.max(0, index);
        return lookAheadIndex;
    }

    public double calculateTargetHeading() {
        return ((pathoperator.targetHeadings.get(lookAheadIndex) + headingOffset) + offset);
    }

    // ---------------- path end ----------------

    public void finishPath() {
        pathFinished = true;
        forceStopPoint = robotPositionVector;
    }

    /** Within 1 cm, or holding (translation stops inside holdT, so a tighter
     *  check could otherwise never pass). */
    public boolean isFinished() {
        Vector2D error = getErrorToPointOnPath(robotPositionVector.getX(), robotPositionVector.getY());
        return Math.abs(error.getX()) < 1 && Math.abs(error.getY()) < 1 || heldXY || pathFinished;
    }

    /** Tolerances below holdT are raised to it, for the same reason. */
    public boolean isFinished(double xTolerance, double yTolerance) {
        xTolerance = Math.max(xTolerance, config.holdT);
        yTolerance = Math.max(yTolerance, config.holdT);
        Vector2D error = getErrorToPointOnPath(robotPositionVector.getX(), robotPositionVector.getY());
        if (!pathFinished && Math.abs(error.getX()) < xTolerance && Math.abs(error.getY()) < yTolerance) {
            pathFinished = true;
            forceStopPoint = robotPositionVector;
        }
        return Math.abs(error.getX()) < xTolerance && Math.abs(error.getY()) < yTolerance || pathFinished;
    }

    public double getErrorToEnd() {
        Vector2D error = getErrorToPointOnPath(robotPositionVector.getX(), robotPositionVector.getY());
        return Math.hypot(error.getX(), error.getY());
    }

    public double getXError() {
        Vector2D endPoint = pathoperator.getPointOnFollowable(pathoperator.getLastPoint());
        return Math.abs(endPoint.getX() - robotPositionVector.getX());
    }

    public double getYError() {
        Vector2D endPoint = pathoperator.getPointOnFollowable(pathoperator.getLastPoint());
        return Math.abs(endPoint.getY() - robotPositionVector.getY());
    }

    /** End point minus (currentX, currentY). */
    public Vector2D getErrorToPointOnPath(double currentX, double currentY) {
        if (pathoperator == null || pathoperator.size() == 0) return new Vector2D();
        Vector2D endPoint = pathoperator.getPointOnFollowable(pathoperator.getLastPoint());
        return new Vector2D(endPoint.getX() - currentX, endPoint.getY() - currentY);
    }

    public Vector2D getErrorToPath() {
        int index = pathoperator.getRobotPositionOnPath(robotPositionVector);
        targetVelocity = pathoperator.getTargetVelocity(index);
        return pathoperator.getErrorToPath(robotPositionVector, index);
    }
}
