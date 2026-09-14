package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;

/**
 * Metrics captured for a single test-path run, and the cost derived from them.
 *
 * Cost policy (lower is better), for the "loose" gate (posTol = 6, hdgTol = 5):
 *   - left the safe box            -> huge penalty (candidate is dangerous)
 *   - timed out / outside gate     -> big penalty scaled by how far off it was
 *   - inside gate                  -> settleTime + overshoot + residual error
 *
 * Speed is rewarded implicitly: a candidate that settles faster (lower
 * settleTime) while staying in the gate wins, so the tuner naturally prefers
 * the most aggressive gains that still land accurately.
 */
public class TrialResult {

    public double settleTime;   // s to reach the gate and stop (== timeout if never)
    public double finalPosErr;  // cm from target at settle (hypot)
    public double finalErrX;    // cm |X - targetX| at settle
    public double finalErrY;    // cm |Y - targetY| at settle
    public double finalHdgErr;  // deg heading error at settle
    public double overshoot;    // cm travelled past the target along the path dir
    public double peakSpeed;    // cm/s max speed seen during the run
    public boolean timedOut;
    public boolean leftBox;

    // Tracking deviation: perpendicular distance from the ideal start->target
    // line, sampled every loop while pathing. This is the "how far off the path
    // am I while driving" signal (vs finalPosErr which is only the endpoint).
    public double sumDev;
    public int    devCount;
    public double maxDev;

    public double avgDev() {
        return devCount > 0 ? sumDev / devCount : 0;
    }

    public double cost(double posTol, double hdgTol) {
        if (leftBox) {
            return 2000;
        }
        boolean outOfGate = timedOut || finalPosErr > posTol || finalHdgErr > hdgTol;
        if (outOfGate) {
            return 500 + finalPosErr * 10 + finalHdgErr * 2;
        }
        // In the gate: minimise time, punish overshoot and residual error.
        return settleTime + 3 * overshoot + 5 * finalPosErr + 0.5 * finalHdgErr;
    }

    public String csv() {
        return String.format("%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%d,%d",
                settleTime, finalPosErr, finalHdgErr, avgDev(), maxDev, overshoot,
                peakSpeed, timedOut ? 1 : 0, leftBox ? 1 : 0);
    }

    public static String csvHeader() {
        return "settleTime,finalPosErr,finalHdgErr,avgDev,maxDev,overshoot,peakSpeed,timedOut,leftBox";
    }
}
