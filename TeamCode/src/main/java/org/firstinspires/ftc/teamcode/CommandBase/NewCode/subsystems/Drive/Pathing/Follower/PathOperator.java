package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.Follower;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathingUtility.PathingVelocity;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.RobotUtilities.Vector2D;

import java.util.ArrayList;
import java.util.List;

/**
 * Geometry of one followable path: closest point, arc length to the end,
 * tangent, signed curvature and the corner speed limit (already slowed in
 * advance so the robot can brake for each corner).
 */
public class PathOperator {

    private static final int searchWindow = 50;
    private static final int TANGENT_SPAN = 4;      // points either side (0.25 cm each)
    private static final int CURVE_SPAN = 8;
    private static final int SEARCH_BACK = 40, SEARCH_AHEAD = 160;

    ArrayList<Vector2D> followablePath;
    ArrayList<PathingVelocity> pathingVelocity;
    ArrayList<Double> targetHeadings;

    int lastPointOnPath;

    private int n;
    private double[] remaining;     // arc length from point i to the end
    private double[] tx, ty;        // unit tangent
    private double[] curvature;     // signed, + = turning toward the left normal (-ty, tx)
    private double[] cornerSpeed;   // max speed at point i, braking for later corners included
    private int closest = 0;

    public PathOperator(ArrayList<Vector2D> followablePath, ArrayList<PathingVelocity> pathingVelocity) {
        this(followablePath, pathingVelocity, new ArrayList<Double>(), 600, 800);
    }

    public PathOperator(ArrayList<Vector2D> followablePath, ArrayList<PathingVelocity> pathingVelocity, ArrayList<Double> targetHeadings) {
        this(followablePath, pathingVelocity, targetHeadings, 600, 800);
    }

    public PathOperator(ArrayList<Vector2D> followablePath, ArrayList<PathingVelocity> pathingVelocity,
                        ArrayList<Double> targetHeadings, double latAccel, double decel) {
        this.followablePath = followablePath;
        this.pathingVelocity = pathingVelocity;
        this.targetHeadings = targetHeadings;
        buildGeometry(latAccel, decel);
    }

    private void buildGeometry(double latAccel, double decel) {
        n = followablePath.size();
        remaining = new double[n];
        tx = new double[n];
        ty = new double[n];
        curvature = new double[n];
        cornerSpeed = new double[n];
        if (n == 0) return;

        for (int i = n - 2; i >= 0; i--) {
            remaining[i] = remaining[i + 1] + dist(followablePath.get(i), followablePath.get(i + 1));
        }
        for (int i = 0; i < n; i++) {
            Vector2D a = followablePath.get(Math.max(0, i - TANGENT_SPAN));
            Vector2D b = followablePath.get(Math.min(n - 1, i + TANGENT_SPAN));
            double dx = b.getX() - a.getX(), dy = b.getY() - a.getY();
            double len = Math.hypot(dx, dy);
            if (len < 1e-9) {
                tx[i] = i > 0 ? tx[i - 1] : 1;
                ty[i] = i > 0 ? ty[i - 1] : 0;
            } else {
                tx[i] = dx / len;
                ty[i] = dy / len;
            }
        }
        for (int i = 0; i < n; i++) {
            int a = Math.max(0, i - CURVE_SPAN), b = Math.min(n - 1, i + CURVE_SPAN);
            double ds = remaining[a] - remaining[b];
            if (ds < 1e-6) continue;
            double cross = tx[a] * ty[b] - ty[a] * tx[b];
            double dot = tx[a] * tx[b] + ty[a] * ty[b];
            curvature[i] = Math.atan2(cross, dot) / ds;
        }
        for (int i = 0; i < n; i++) {
            double k = Math.abs(curvature[i]);
            cornerSpeed[i] = k > 1e-6 ? Math.sqrt(latAccel / k) : Double.MAX_VALUE;
        }
        for (int i = n - 2; i >= 0; i--) {
            double ds = remaining[i] - remaining[i + 1];
            double next = cornerSpeed[i + 1];
            if (next < Double.MAX_VALUE) {
                cornerSpeed[i] = Math.min(cornerSpeed[i], Math.sqrt(next * next + 2 * decel * ds));
            }
        }
    }

    private static double dist(Vector2D a, Vector2D b) {
        return Math.hypot(b.getX() - a.getX(), b.getY() - a.getY());
    }

    public int size() { return n; }
    public double remaining(int i) { return remaining[i]; }
    public double tangentX(int i) { return tx[i]; }
    public double tangentY(int i) { return ty[i]; }
    public double curvature(int i) { return curvature[i]; }
    public double cornerSpeed(int i) { return cornerSpeed[i]; }

    /** Closest path point to (x, y), searched in a window around the last result. */
    public int closestIndex(double x, double y) {
        if (n == 0) return 0;
        int from = Math.max(0, closest - SEARCH_BACK), to = Math.min(n, closest + SEARCH_AHEAD);
        int best = closest;
        double bestD = Double.MAX_VALUE;
        for (int i = from; i < to; i++) {
            Vector2D p = followablePath.get(i);
            double d = (p.getX() - x) * (p.getX() - x) + (p.getY() - y) * (p.getY() - y);
            if (d < bestD) {
                bestD = d;
                best = i;
            }
        }
        closest = best;
        return best;
    }

    /** Full search, for (re)starting a path from anywhere. */
    public int resetClosest(double x, double y) {
        closest = 0;
        double bestD = Double.MAX_VALUE;
        for (int i = 0; i < n; i++) {
            Vector2D p = followablePath.get(i);
            double d = (p.getX() - x) * (p.getX() - x) + (p.getY() - y) * (p.getY() - y);
            if (d < bestD) {
                bestD = d;
                closest = i;
            }
        }
        return closest;
    }

    // ---------------- original Nexus methods ----------------

    public double getPathHeading(PathingVelocity targetVelocity) {
        double radians = Math.atan2(targetVelocity.getYVelocity(), targetVelocity.getXVelocity());
        double degrees = Math.toDegrees(radians);
        if (degrees < 0) degrees += 360;
        return degrees;
    }

    public double pathLength() {
        return n > 0 ? remaining[0] : 0;
    }

    public double calculateDistancePointToPoint(Vector2D robotPos, Vector2D endpoint) {
        int startIndex = getRobotPositionOnPathFullPath(robotPos);
        int endIndex = getRobotPositionOnPathFullPath(endpoint);
        List<Vector2D> sectionToLook = followablePath.subList(startIndex, endIndex);
        double totalDistance = 0.0;
        for (int i = 0; i < sectionToLook.size() - 1; i++) {
            totalDistance += dist(sectionToLook.get(i), sectionToLook.get(i + 1));
        }
        return totalDistance;
    }

    public int getRobotPositionOnPath(Vector2D robotPos) {
        int index = lastPointOnPath;
        double minDistance = Double.MAX_VALUE;
        int searchStart = Math.max(0, lastPointOnPath - (int) (searchWindow * 0.2));
        int searchEnd = Math.min(followablePath.size(), (lastPointOnPath + (int) (searchWindow * 0.8)));
        for (int i = searchStart; i < searchEnd; i++) {
            Vector2D pos = followablePath.get(i);
            double distance = Math.hypot(robotPos.getX() - pos.getX(), robotPos.getY() - pos.getY());
            if (distance < minDistance) {
                minDistance = distance;
                index = i;
            }
        }
        lastPointOnPath = index;
        if (index + 10 <= followablePath.size() - 1) index += 10;
        return index;
    }

    public int getRobotPositionOnPathFullPath(Vector2D robotPos) {
        int index = 0;
        double minDistance = Double.MAX_VALUE;
        for (int i = 0; i < followablePath.size(); i++) {
            Vector2D pos = followablePath.get(i);
            double distance = Math.hypot(robotPos.getX() - pos.getX(), robotPos.getY() - pos.getY());
            if (distance < minDistance) {
                minDistance = distance;
                index = i;
            }
        }
        lastPointOnPath = index;
        return index;
    }

    public Vector2D getErrorToPath(Vector2D robotPos, int index) {
        Vector2D position = followablePath.get(Math.min(index, followablePath.size() - 1));
        return new Vector2D(position.getX() - robotPos.getX(), position.getY() - robotPos.getY());
    }

    public PathingVelocity getTargetVelocity(int index) {
        PathingVelocity targetVelocity = new PathingVelocity();
        if (index > pathingVelocity.size() - 1) {
            targetVelocity.set(0, 0);
        } else {
            targetVelocity.set(pathingVelocity.get(index).getXVelocity(), pathingVelocity.get(index).getYVelocity());
        }
        return targetVelocity;
    }

    public Vector2D getPointOnFollowable(int index) {
        return followablePath.get(index);
    }

    public int getLastPoint() {
        return followablePath.size() == 0 ? 0 : followablePath.size() - 1;
    }
}
