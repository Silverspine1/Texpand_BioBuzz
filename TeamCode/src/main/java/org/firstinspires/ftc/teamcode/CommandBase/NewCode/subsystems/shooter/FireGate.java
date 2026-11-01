package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

/**
 * Fire-timing gate (review R4 + R5 turret rule), a line-by-line port of
 * {@code ftc_optimizer.shotcontrol.algos.fire_gate} (spec: docs/SHOT_ALGO_DESIGN.md section 7.1 item 6 and 5.2).
 * Every control tick: release the queued shot now, or wait.
 *
 * <p>Inputs: {@code feasible} / {@code pNow} of the solve at the current estimated pose, {@code pAhead[j]} = claim
 * of the same solve at the pose propagated by {@link #HORIZONS_S}[j] with constant acceleration / turn rate (0 when
 * infeasible), {@code tLeft} = time left to the driver's deadline [s] (horizons beyond it are ignored), turret
 * command / measured angle [deg], measured turret rate and the command's own rate [deg/s], flywheel command /
 * measured rpm and the command's rate [rpm/s], yaw rate [deg/s], {@code deadline}.
 *
 * <p>Rules, in order: (a) infeasible -> {@link #WAIT_INFEASIBLE} (even at the deadline); (a2) {@link #protect} false
 * (movement protection OFF) -> {@link #FIRE}; (b) deadline -> {@link #FIRE}; (c) actuators not where the command will
 * be when the ball passes the turret -> {@link #WAIT_UNSETTLED}: the turret error PREDICTED at release,
 * {@code e_rel = wrap180(cmd - meas) + (targetGain * cmdRate - rate) * tReleaseS}, must be within {@link #turRelTolDeg}
 * (a turret 5 deg off but closing fast fires; one on position but moving fast away does not); (d) a look-ahead claim
 * within the window beats the current one by more than {@code lookEps} -> {@link #WAIT_BETTER}; else {@link #FIRE}.
 *
 * <p>Scalar, no allocation, no SDK. The thresholds are instance fields initialised to the Python constants (the
 * parity test uses the defaults; the robot may tune them).
 */
public final class FireGate {

    /** fire_gate.HORIZONS_S [s]. */
    public static final double[] HORIZONS_S = {0.04, 0.08, 0.12, 0.20, 0.30};
    /** fire_gate.T_RELEASE_S: decision to ball pass-through [s] (= ErrorConfig.fire_latency_s; assumed, not measured). */
    public static final double T_RELEASE_S = 0.080;
    /** fire_gate.TUR_REL_TOL_DEG: refuse when the predicted turret error at release exceeds this [deg]. */
    public static final double TUR_REL_TOL_DEG = 6.0;
    /** fire_gate.TUR_ERR_DEG: sanity limit on the current error [deg]. */
    public static final double TUR_ERR_DEG = 20.0;
    /** fire_gate.TUR_RATE_CAP_DPS: refuse above this turret speed [deg/s]. */
    public static final double TUR_RATE_CAP_DPS = 130.0;
    /** fire_gate.TUR_TARGET_GAIN: weight of the turret command's own rate in e_rel (0: the command already leads). */
    public static final double TUR_TARGET_GAIN = 0.0;
    public static final double RPM_TOL = 40.0;
    public static final double RPM_STATIC_RPS = 400.0;
    public static final double OMEGA_MAX_DPS = 250.0;
    public static final double LOOK_EPS = 0.03;

    /** Decisions (fire_gate.FIRE, WAIT_INFEASIBLE, WAIT_UNSETTLED, WAIT_BETTER). */
    public static final int FIRE = 0, WAIT_INFEASIBLE = 1, WAIT_UNSETTLED = 2, WAIT_BETTER = 3;
    /** refuse_reason codes: 0 settled, 1 turret error now, 2 turret error at release, 3 flywheel, 4 yaw rate, 5 turret speed. */
    public static final int SETTLED = 0, REFUSE_TURRET_ERR = 1, REFUSE_TURRET_REL = 2, REFUSE_RPM = 3,
            REFUSE_YAW_RATE = 4, REFUSE_TURRET_RATE = 5;

    /** Movement protection (the settle gate). false: any feasible solve fires at once. Default ON. */
    public boolean protect = true;
    public double turRelTolDeg = TUR_REL_TOL_DEG;
    public double turErrDeg = TUR_ERR_DEG;
    public double turRateCapDps = TUR_RATE_CAP_DPS;
    public double targetGain = TUR_TARGET_GAIN;
    public double tReleaseS = T_RELEASE_S;
    public double rpmTol = RPM_TOL;
    public double rpmStaticRps = RPM_STATIC_RPS;
    public double omegaMaxDps = OMEGA_MAX_DPS;
    public double lookEps = LOOK_EPS;

    /** Diagnostics of the last {@link #decide} call: refuse reason (0 when not evaluated), best look-ahead claim
     *  and the predicted turret error at release {@link #turretReleaseError} (NaN when not evaluated). */
    public int lastRefuse;
    public double lastBest;
    public double lastEstRel = Double.NaN;

    /** fire_gate.wrap180: [-180, 180); NaN / infinity pass through. */
    public static double wrap180(double a) {
        if (Double.isNaN(a) || Double.isInfinite(a)) {
            return a;
        }
        return a - 360.0 * Math.floor((a + 180.0) / 360.0);
    }

    /** fire_gate.turret_release_error: predicted turret aiming error at ball pass-through [deg]. */
    public double turretReleaseError(double turCmd, double turMeas, double turRate, double turCmdRate) {
        return wrap180(turCmd - turMeas) + (targetGain * turCmdRate - turRate) * tReleaseS;
    }

    /** fire_gate.refuse_reason: 0 = settled; 1 turret error now; 2 turret error at release; 3 flywheel; 4 yaw rate; 5 turret speed. */
    public int refuseReason(double turCmd, double turMeas, double turRate, double turCmdRate,
                            double rpmCmd, double rpmMeas, double rpmCmdRate, double omega) {
        if (Math.abs(wrap180(turCmd - turMeas)) > turErrDeg) {
            return REFUSE_TURRET_ERR;
        }
        lastEstRel = turretReleaseError(turCmd, turMeas, turRate, turCmdRate);
        if (Math.abs(lastEstRel) > turRelTolDeg) {
            return REFUSE_TURRET_REL;
        }
        if (Math.abs(turRate) > turRateCapDps) {
            return REFUSE_TURRET_RATE;
        }
        if (Math.abs(rpmCmdRate) <= rpmStaticRps && Math.abs(rpmCmd - rpmMeas) > rpmTol) {
            return REFUSE_RPM;
        }
        if (Math.abs(omega) > omegaMaxDps) {
            return REFUSE_YAW_RATE;
        }
        return SETTLED;
    }

    /** fire_gate.fire_gate with the default horizons. */
    public int decide(boolean feasible, double pNow, double[] pAhead, double tLeft, double turCmd, double turMeas,
                      double turRate, double turCmdRate, double rpmCmd, double rpmMeas, double rpmCmdRate,
                      double omega, boolean deadline) {
        return decide(feasible, pNow, pAhead, tLeft, turCmd, turMeas, turRate, turCmdRate, rpmCmd, rpmMeas,
                rpmCmdRate, omega, deadline, HORIZONS_S);
    }

    /** fire_gate.fire_gate: FIRE or a WAIT_* code. */
    public int decide(boolean feasible, double pNow, double[] pAhead, double tLeft, double turCmd, double turMeas,
                      double turRate, double turCmdRate, double rpmCmd, double rpmMeas, double rpmCmdRate,
                      double omega, boolean deadline, double[] horizons) {
        lastRefuse = SETTLED;
        lastBest = 0.0;
        lastEstRel = turretReleaseError(turCmd, turMeas, turRate, turCmdRate);
        if (!feasible) {
            return WAIT_INFEASIBLE;
        }
        if (!protect) {
            return FIRE;
        }
        if (deadline) {
            return FIRE;
        }
        lastRefuse = refuseReason(turCmd, turMeas, turRate, turCmdRate, rpmCmd, rpmMeas, rpmCmdRate, omega);
        if (lastRefuse != SETTLED) {
            return WAIT_UNSETTLED;
        }
        double best = 0.0;
        for (int j = 0; j < horizons.length; j++) {
            if (horizons[j] <= tLeft && pAhead[j] > best) {
                best = pAhead[j];
            }
        }
        lastBest = best;
        if (best > pNow + lookEps) {
            return WAIT_BETTER;
        }
        return FIRE;
    }
}
