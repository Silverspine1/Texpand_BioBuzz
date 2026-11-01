package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

/**
 * Turns the {@link FireGate} decision into a FEED permission: the "hold the shot until the gate says FIRE" logic of
 * Test_tele and ShotTuner. Pure Java (no SDK), no allocation, tested in tools/shot_java_parity/ShotTunerTest.java.
 *
 * <p>Per driver trigger press (the feed button, gamepad1 left bumper):
 * <ul>
 * <li>gate not applicable (reduced controller off, no model, {@code ShotControlTuning.gateHoldsFire} false): the
 *     trigger passes straight through (the old behaviour), state {@link #PASS};</li>
 * <li>rising edge: {@link #consumeRequestOpened()} is true once, so the caller opens the gate's window
 *     ({@code ReducedShotController.requestShot}); the window's deadline makes a FEASIBLE solve fire even if the
 *     actuators never settle;</li>
 * <li>the manual override ({@code override}) feeds at once, whatever the gate says (state {@link #OVERRIDE});</li>
 * <li>otherwise the feed is held ({@link #HOLDING}) until the gate says {@link FireGate#FIRE} (state
 *     {@link #RELEASED}); once released the feed stays on while the trigger is held (a burst is not re-gated per
 *     ball); letting go of the trigger resets everything. An infeasible solve holds indefinitely (use the override).</li>
 * </ul>
 * Movement protection OFF is not handled here: with {@code FireGate.protect == false} the gate returns FIRE for any
 * feasible solve, so the hold only waits for feasibility.
 */
public final class ShotHold {

    public static final int IDLE = 0, PASS = 1, HOLDING = 2, RELEASED = 3, OVERRIDE = 4;
    public static final String[] TEXT = {"idle", "pass-through", "HOLDING for the gate", "released by the gate", "OVERRIDE"};

    public int state = IDLE;
    /** Time the current trigger press started [s]. */
    public double pressedS;
    /** True on the call that STARTED feeding for this press (one shot event per press); cleared by the next call. */
    public boolean feedStarted;
    private boolean requestOpened;

    /**
     * @param nowS            monotonic time [s]
     * @param trigger         the driver's feed button is down
     * @param override        manual override button is down (feed regardless of the gate)
     * @param gateApplicable  the reduced controller is the active command source and the hold is enabled
     * @param gateState       FireGate decision of this loop (FIRE, WAIT_*); ignored when not applicable
     * @return true = feed now
     */
    public boolean update(double nowS, boolean trigger, boolean override, boolean gateApplicable, int gateState) {
        feedStarted = false;
        if (!trigger) {
            state = IDLE;
            return false;
        }
        if (state == IDLE) {
            pressedS = nowS;
            requestOpened = gateApplicable;
            state = gateApplicable ? HOLDING : PASS;
            if (!gateApplicable) {
                feedStarted = true;
                return true;
            }
        }
        if (!gateApplicable) {
            if (state != PASS) feedStarted = true;
            state = PASS;
            return true;
        }
        if (state == PASS) {                       // the gate became applicable mid-press: treat it as a new press
            state = HOLDING;
            pressedS = nowS;
            requestOpened = true;
        }
        if (state == RELEASED || state == OVERRIDE) {
            return true;
        }
        if (override) {
            state = OVERRIDE;
            feedStarted = true;
            return true;
        }
        if (gateState == FireGate.FIRE) {
            state = RELEASED;
            feedStarted = true;
            return true;
        }
        state = HOLDING;
        return false;
    }

    /** True once after a press started while the gate applies: the caller opens the gate's request window. */
    public boolean consumeRequestOpened() {
        boolean r = requestOpened;
        requestOpened = false;
        return r;
    }

    /** Seconds the current press has been held back (0 when not holding). */
    public double heldBackS(double nowS) {
        return state == HOLDING ? nowS - pressedS : 0.0;
    }

    public void reset() {
        state = IDLE;
        feedStarted = false;
        requestOpened = false;
    }
}
