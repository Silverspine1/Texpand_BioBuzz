package org.firstinspires.ftc.teamcode.CommandBase.NewCode;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Localisation.Odometry;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.ReachabilityMap;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.ShooterController;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.Shotplanner;

/**
 * Draws the field on FTC Dashboard: goals, where the planner can make a shot from, the robot, and where
 * the turret is aiming. Field metres (origin at your field corner) map onto Dashboard's centred inches;
 * use fieldViewRotationDeg / fieldViewFlipY in ShooterTuning if the picture is turned or mirrored.
 */
public class TargetingDebugView {

    private static final double FIELD_M = 3.6576;
    private static final double IN_PER_M = 39.3701;

    private final ReachabilityMap map = new ReachabilityMap(FIELD_M);

    private double cosR = 1;
    private double sinR = 0;
    private boolean flip = false;

    public void draw(TelemetryPacket packet, ShooterController shooter, Odometry odometry) {
        double r = Math.toRadians(ShooterTuning.fieldViewRotationDeg);
        cosR = Math.cos(r);
        sinR = Math.sin(r);
        flip = ShooterTuning.fieldViewFlipY;

        Canvas canvas = packet.fieldOverlay();
        Shotplanner.HiveTarget hive = shooter.getHiveTarget();

        drawBorder(canvas);
        drawReachable(canvas, shooter, hive);
        drawGoal(canvas, ShooterController.getHiveTarget(Shotplanner.BlueHiveSide.AUDIENCE), shooter.blueHiveSide == Shotplanner.BlueHiveSide.AUDIENCE);
        drawGoal(canvas, ShooterController.getHiveTarget(Shotplanner.BlueHiveSide.OPPOSITE), shooter.blueHiveSide == Shotplanner.BlueHiveSide.OPPOSITE);

        double x = odometry.X() / 100;
        double y = odometry.Y() / 100;
        double heading = odometry.Heading();
        drawRobot(canvas, x, y, heading);

        drawRay(canvas, x, y, Math.toDegrees(Math.atan2(hive.openingY - y, hive.openingX - x)),
                Math.hypot(hive.openingX - x, hive.openingY - y), "#9e9e9e", 1, 0.8);
        drawRay(canvas, x, y, heading + shooter.getTurretAimDeg(), 1.6, "#00bcd4", 3, 1.0);

        Shotplanner.ShotSolution shot = shooter.lastShot;
        if (shot != null && shot.reachable) {
            drawRay(canvas, x, y, heading + shot.turretAngleDeg, 1.3, "#ff9800", 2, 1.0);
        }

        packet.put("goal distance (m)", Math.hypot(hive.openingX - x, hive.openingY - y));
        packet.put("goal bearing (turret deg)", shooter.getGoalBearingDeg());
        packet.put("map", map.isComputing() ? "computing" : "ready");
        if (shot != null) {
            packet.put("shot", shot.reachable ? (shot.safe ? "reachable" : "reachable (tight)") : "NO SHOT: " + shot.failureReason);
        }
    }

    private void drawBorder(Canvas canvas) {
        canvas.setStroke("#757575").setStrokeWidth(1).setAlpha(1.0);
        canvas.strokePolygon(
                new double[]{fx(0, 0), fx(FIELD_M, 0), fx(FIELD_M, FIELD_M), fx(0, FIELD_M)},
                new double[]{fy(0, 0), fy(FIELD_M, 0), fy(FIELD_M, FIELD_M), fy(0, FIELD_M)});
    }

    private void drawReachable(Canvas canvas, ShooterController shooter, Shotplanner.HiveTarget hive) {
        ReachabilityMap.Snapshot snapshot = map.update(
                shooter.getPlannerConfig(), hive, ShooterTuning.mapCellMeters, ShooterTuning.plannerDelaySeconds);
        if (snapshot == null) return;

        double cell = snapshot.cell;
        for (int iy = 0; iy < snapshot.size; iy++) {
            for (int ix = 0; ix < snapshot.size; ix++) {
                byte state = snapshot.at(ix, iy);
                if (state == ReachabilityMap.NONE) continue;

                double x0 = ix * cell, x1 = x0 + cell, y0 = iy * cell, y1 = y0 + cell;
                canvas.setFill(state == ReachabilityMap.SHOT ? "#4caf50" : "#ffb300").setAlpha(0.35);
                canvas.fillPolygon(
                        new double[]{fx(x0, y0), fx(x1, y0), fx(x1, y1), fx(x0, y1)},
                        new double[]{fy(x0, y0), fy(x1, y0), fy(x1, y1), fy(x0, y1)});
            }
        }
        canvas.setAlpha(1.0);
    }

    private void drawGoal(Canvas canvas, Shotplanner.HiveTarget goal, boolean selected) {
        double face = Math.toRadians(goal.facingAngleDeg);
        double rightX = Math.sin(face), rightY = -Math.cos(face);
        double normalX = Math.cos(face), normalY = Math.sin(face);
        double half = 0.254;

        canvas.setStroke(selected ? "#e53935" : "#9e9e9e").setStrokeWidth(selected ? 4 : 2).setAlpha(1.0);
        canvas.strokeLine(
                fx(goal.openingX - rightX * half, goal.openingY - rightY * half),
                fy(goal.openingX - rightX * half, goal.openingY - rightY * half),
                fx(goal.openingX + rightX * half, goal.openingY + rightY * half),
                fy(goal.openingX + rightX * half, goal.openingY + rightY * half));

        double tipX = goal.openingX + normalX * 0.45, tipY = goal.openingY + normalY * 0.45;
        canvas.setStrokeWidth(1);
        canvas.strokeLine(fx(goal.openingX, goal.openingY), fy(goal.openingX, goal.openingY), fx(tipX, tipY), fy(tipX, tipY));
        canvas.setFill(selected ? "#e53935" : "#9e9e9e");
        canvas.fillCircle(fx(tipX, tipY), fy(tipX, tipY), 2.5);
    }

    private void drawRobot(Canvas canvas, double x, double y, double headingDeg) {
        double h = Math.toRadians(headingDeg);
        double half = ShooterTuning.robotSizeMeters / 2;
        double[] cx = new double[4];
        double[] cy = new double[4];
        double[][] corners = {{half, half}, {half, -half}, {-half, -half}, {-half, half}};
        for (int i = 0; i < 4; i++) {
            double fxr = corners[i][0] * Math.cos(h) - corners[i][1] * Math.sin(h);
            double fyr = corners[i][0] * Math.sin(h) + corners[i][1] * Math.cos(h);
            cx[i] = fx(x + fxr, y + fyr);
            cy[i] = fy(x + fxr, y + fyr);
        }
        canvas.setFill("#1e88e5").setAlpha(0.45).fillPolygon(cx, cy);
        canvas.setAlpha(1.0).setStroke("#1e88e5").setStrokeWidth(2).strokePolygon(cx, cy);
        drawRay(canvas, x, y, headingDeg, half + 0.15, "#1e88e5", 3, 1.0);
    }

    private void drawRay(Canvas canvas, double x, double y, double angleDeg, double length, String color, int width, double alpha) {
        double a = Math.toRadians(angleDeg);
        double ex = x + length * Math.cos(a), ey = y + length * Math.sin(a);
        canvas.setStroke(color).setStrokeWidth(width).setAlpha(alpha);
        canvas.strokeLine(fx(x, y), fy(x, y), fx(ex, ey), fy(ex, ey));
        canvas.setAlpha(1.0);
    }

    private double fx(double x, double y) {
        double ex = (x - FIELD_M / 2) * IN_PER_M;
        double ey = ((flip ? FIELD_M - y : y) - FIELD_M / 2) * IN_PER_M;
        return ex * cosR - ey * sinR;
    }

    private double fy(double x, double y) {
        double ex = (x - FIELD_M / 2) * IN_PER_M;
        double ey = ((flip ? FIELD_M - y : y) - FIELD_M / 2) * IN_PER_M;
        return ex * sinR + ey * cosR;
    }
}
