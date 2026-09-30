package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

/**
 * Keeps acting on the last good shot for a few solves when the planner briefly loses the solution, so a shot sitting
 * on the edge of the reachable area doesn't flip the turret and flywheel between two different commands.
 * The held turret angle is corrected for how far the robot has turned since, so it keeps pointing at the same spot.
 */
public class ShotSolutionHold {

    public int holdSolves = 4;

    private Shotplanner.ShotSolution lastGood;
    private double captureHeadingDeg;
    private int missed = 0;

    public void reset() {
        lastGood = null;
        missed = 0;
    }

    /** Returns the solution to act on: the fresh one if it has a shot, the held one for a few misses, else the fresh one. */
    public Shotplanner.ShotSolution apply(Shotplanner.ShotSolution fresh, double currentHeadingDeg) {
        if (fresh.reachable) {
            lastGood = fresh;
            captureHeadingDeg = currentHeadingDeg;
            missed = 0;
            return fresh;
        }

        missed++;
        if (lastGood != null && missed <= holdSolves) {
            Shotplanner.ShotSolution held = new Shotplanner.ShotSolution();
            held.reachable = lastGood.reachable;
            held.safe = lastGood.safe;
            held.launchAngleDeg = lastGood.launchAngleDeg;
            held.flywheelRPM = lastGood.flywheelRPM;
            held.muzzleSpeedMetersPerSecond = lastGood.muzzleSpeedMetersPerSecond;
            held.flightTimeSeconds = lastGood.flightTimeSeconds;
            held.minimumClearanceMeters = lastGood.minimumClearanceMeters;
            held.releaseHeadingDeg = lastGood.releaseHeadingDeg;
            held.turretAngleDeg = wrap(lastGood.turretAngleDeg - (currentHeadingDeg - captureHeadingDeg));
            return held;
        }
        lastGood = null;
        return fresh;
    }

    private static double wrap(double angle) {
        angle %= 360.0;
        if (angle > 180) angle -= 360;
        if (angle <= -180) angle += 360;
        return angle;
    }
}
