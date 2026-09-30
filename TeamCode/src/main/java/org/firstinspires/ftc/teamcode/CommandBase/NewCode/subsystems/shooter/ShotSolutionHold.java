package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

/**
 * Keeps acting on the last good shot for a few solves when the planner briefly loses the solution, so a shot sitting
 * on the edge of the reachable area doesn't flip the turret and flywheel between two different commands.
 */
public class ShotSolutionHold {

    public int holdSolves = 4;

    private Shotplanner.ShotSolution lastGood;
    private int missed = 0;

    public void reset() {
        lastGood = null;
        missed = 0;
    }

    /** Returns the solution to act on: the fresh one if it has a shot, the held one for a few misses, else the fresh one. */
    public Shotplanner.ShotSolution apply(Shotplanner.ShotSolution fresh) {
        if (fresh.reachable) {
            lastGood = fresh;
            missed = 0;
            return fresh;
        }

        missed++;
        if (lastGood != null && missed <= holdSolves) {
            return lastGood;
        }
        lastGood = null;
        return fresh;
    }
}
