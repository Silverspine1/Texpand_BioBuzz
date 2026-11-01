package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

/**
 * The recommended 4-position field campaign of docs/UNDER_SHOOTER_TUNING.md section 6, as a step list the ShotTuner
 * walks through. Pure Java, no SDK, tested in tools/shot_java_parity/ShotTunerTest.java.
 *
 * <p>Four rounds; each round shoots one ball at each of the four stands (nominal wheel-to-goal range 45, 62, 80 and
 * 92 in, aiming normally with the rpm and hood the controller commands there: the table gets tested too) and then one
 * SLIP shot (5700 rpm, hood 0, at the longest stand) as the wheel-slip check: 4 x (4 + 1) = 20 shots. Order inside a
 * round: the long stands carry the most information, so every round visits all of them. The CLI
 * ({@code field_tune next shots.csv}) can recommend a different next shot after a fit; the tuner only walks the list.
 */
public final class ShotCampaign {

    public static final int[] STANDS_IN = {45, 62, 80, 92};
    public static final int ROUNDS = 4;
    public static final double SLIP_RPM = 5700.0, SLIP_HOOD_DEG = 0.0;
    /** The wheel-to-goal range counts as "at the stand" within this many inches of the nominal range. The CSV range_in is
     *  the turret axis (robot reference point) to the aim point; field_tune subtracts the wheel offset WHEEL_FORWARD_IN
     *  (the wheel axis is that far ahead of the reference point), so wheel range = range_in - WHEEL_FORWARD_IN. */
    public static final double AT_STAND_TOL_IN = 6.0;
    public static final double WHEEL_FORWARD_IN = 4.0;

    public static final int STAND = 0, SLIP = 1;

    private final int[] kind;
    private final int[] nominal;
    private int index = 0;
    private int labelled = 0;

    public ShotCampaign() {
        int n = ROUNDS * (STANDS_IN.length + 1);
        kind = new int[n];
        nominal = new int[n];
        int k = 0;
        for (int r = 0; r < ROUNDS; r++) {
            for (int s = 0; s < STANDS_IN.length; s++, k++) {
                kind[k] = STAND;
                nominal[k] = STANDS_IN[s];
            }
            kind[k] = SLIP;
            nominal[k] = STANDS_IN[STANDS_IN.length - 1];
            k++;
        }
    }

    public int size() {
        return kind.length;
    }

    public int index() {
        return index;
    }

    public int labelledCount() {
        return labelled;
    }

    public boolean done() {
        return index >= kind.length;
    }

    public boolean isSlip() {
        return !done() && kind[index] == SLIP;
    }

    /** Nominal wheel-to-goal range of the current step [in]; 0 when done. */
    public int nominalRangeIn() {
        return done() ? 0 : nominal[index];
    }

    /** Round (1-based) of the current step. */
    public int round() {
        return done() ? ROUNDS : index / (STANDS_IN.length + 1) + 1;
    }

    /** position_id for the CSV: "P45", or "P92S" for the slip shot. */
    public String positionId() {
        if (done()) return "";
        return ShotCsv.positionId(nominal[index]) + (kind[index] == SLIP ? "S" : "");
    }

    /** True when (turret-axis-to-aim range - wheel offset) is within {@link #AT_STAND_TOL_IN} of the current stand. */
    public boolean atStand(double rangeIn) {
        if (done() || Double.isNaN(rangeIn)) return false;
        return Math.abs(rangeIn - WHEEL_FORWARD_IN - nominal[index]) <= AT_STAND_TOL_IN;
    }

    /** A shot of the current step was taken and labelled: advance. */
    public void shotLabelled() {
        if (done()) return;
        labelled++;
        index++;
    }

    /** Skip the current step without a shot (e.g. the position is not reachable). */
    public void skip() {
        if (!done()) index++;
    }

    /** Step back one (a mis-press). */
    public void back() {
        if (index > 0) index--;
    }

    public void restart() {
        index = 0;
        labelled = 0;
    }

    /** One-line instruction for the telemetry. */
    public String instruction() {
        if (done()) {
            return "CAMPAIGN DONE: " + labelled + " shots. Pull the CSV and run field_tune fit (docs/SHOT_CONTROL_README.md).";
        }
        String head = "Round " + round() + "/" + ROUNDS + "  step " + (index + 1) + "/" + kind.length + ": ";
        if (kind[index] == SLIP) {
            return head + "SLIP check: stand at ~" + nominal[index] + " in, shoot ONE ball (the tuner forces "
                    + (int) SLIP_RPM + " rpm, hood " + (int) SLIP_HOOD_DEG + ", override on), then label it.";
        }
        return head + "stand at ~" + nominal[index] + " in wheel-to-goal (aim normally), shoot ONE ball, then label it.";
    }
}
