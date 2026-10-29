package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathGeneration;


import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.RobotUtilities.Vector2D;

public class SegmentGenerator {

    private Vector2D onTheCurve;

    private ArrayList<Vector2D> Segment = new ArrayList<>();

    private ArrayList<Vector2D> curvaturePoints = new ArrayList<>();

    double t = 0.0;

    public ArrayList<Vector2D> copyPath(){
        return Segment;
    }

    public ArrayList<Vector2D> copyCurvature(){
        return curvaturePoints;
    }

    public void buildPath(Vector2D startPoint, Vector2D endPoint){

        Segment.clear();
        t = 0;

        do{
            onTheCurve = calculateLine(startPoint, endPoint, t);

            t += 0.001;

            Segment.add(onTheCurve);

        }while (t <= 1.0);

    }

    public void buildPath(Vector2D startPoint, Vector2D controlPoint, Vector2D endPoint){

        Segment.clear();
        t = 0;

        do{
            onTheCurve = calculateQuadraticBezier(startPoint, controlPoint, endPoint, t);

            t += 0.001;

            Segment.add(onTheCurve);

        }while (t <= 1.0);

    }

    public void buildPath( Vector2D startPoint, Vector2D controlPoint1, Vector2D controlPoint2, Vector2D endPoint){

        Segment.clear();
        t = 0;

        do{
            onTheCurve = calculateQuadraticBezier(startPoint, controlPoint1, controlPoint2, endPoint, t);

            t += 0.001;

            Segment.add(onTheCurve);

        }while (t <= 1.0);

    }

    private Vector2D calculateQuadraticBezier(Vector2D start, Vector2D control, Vector2D end, double t) {
        double u = 1 - t;
        double tt = t * t;
        double uu = u * u;

        double x = uu * start.getX() + 2 * u * t * control.getX() + tt * end.getX();
        double y = uu * start.getY() + 2 * u * t * control.getY() + tt * end.getY();

        return new Vector2D(x, y);
    }

    private Vector2D calculateQuadraticBezier(Vector2D start, Vector2D controlFirst, Vector2D controlSecond, Vector2D end, double t) {
        double u = 1 - t;
        double tt = t * t;
        double ttt = t * t * t;
        double uu = u * u;
        double uuu = u * u * u;

        double x = uuu * start.getX() + 3 * uu * t * controlFirst.getX() + 3 * u * tt * controlSecond.getX() + ttt * end.getX();
        double y = uuu * start.getY() + 3 * uu * t * controlFirst.getY() + 3 * u * tt * controlSecond.getY() + ttt * end.getY();

        return new Vector2D(x, y);
    }

    private Vector2D calculateLine(Vector2D start, Vector2D end, double t) {
        double u = 1 - t;

        double x = start.getX() * u + end.getX() * t;
        double y = start.getY() * u + end.getY() * t;

        return new Vector2D(x, y);
    }

}
