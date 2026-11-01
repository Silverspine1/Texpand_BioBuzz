package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

/**
 * Robot-side wrapper of {@link ReducedTargeter}: frame conversion ({@link ShotFrame}), turret-pivot override,
 * battery rpm ceiling, burst bookkeeping (shot index + release spacing), driver trims, the R4 fire gate
 * ({@link FireGate} with the 5-horizon look-ahead) and the solve-time measurement.
 * Pure Java (no FTC SDK): {@code ShooterController} feeds it the sensor values and applies the commands, so the
 * whole chain runs in a plain JVM for the tests. No allocation after construction.
 *
 * <p>Turret: the output {@link #turretJavaDeg} is in the existing Java planner convention (relative to
 * {@code Odometry.Heading()}, before {@code ShooterTuning.turretZeroOffsetDeg}); the existing {@code SetTurretAngle}
 * controller drives the turret exactly as before.
 *
 * <p>Fire gate: this class only DECIDES ({@link #gateState}, {@link #gateOpen}); nothing here releases a ball ({@link ShotHold}
 * and the OpMode do). {@link Params#gateProtect} false = movement protection OFF: FIRE whenever feasible. The driver's
 * window starts with {@link #requestShot}; without a request the gate is evaluated as if one had just started
 * ({@code t_left = gateWindowS}, no deadline), i.e. "would it fire now".
 */
public final class ReducedShotController {

    /** Tunables (copied from {@code ShotControlTuning} by the subsystem each loop; plain fields here). */
    public static final class Params {
        public boolean blueAlliance = true;
        /** true: turret axis from ShooterTuning (offset from the odometry point); false: the model's constants. */
        public boolean pivotFromRobot = true;
        public double pivotForwardM = 0.1, pivotSidewaysM = 0.0;
        /** Keep the model's bearing-rate turret lead (turret_lead_s, tuned on the sim's turret model). */
        public boolean useModelTurretLead = true;
        public double speedTrimPct = 0.0, turretTrimDeg = 0.0, hoodTrimDeg = 0.0;
        /** FIELD TUNING 1: real wheel rpm = model rpm x rpmScale (the real wheel throws the ball faster per rpm than the
         *  contact sim says when < 1). 1 = Python parity. */
        public double rpmScale = 1.0;
        /** FIELD TUNING 2: the real exit angle minus the model's [deg] (+ = real shot steeper). 0 = Python parity. */
        public double angleOffsetDeg = 0.0;
        /** Battery-limited wheel ceiling [rpm] (R4: ~0.98 x no-load rpm x V/12); <= 0 = the model's rpm_max.
         *  The solve clamps to min(model rpm_max, this): a state that needs more becomes infeasible, claim 0. */
        public double rpmMaxCeiling = 0.0;
        /** Mechanism delay [s]: the rpm and hood commands are solved for the state this far ahead (constant acceleration and
         *  turn rate), so the wheel and hood are there when the robot is. The turret keeps its own lead (SetTurretAngle).
         *  0 = Python parity. */
        public double mechLeadS = 0.0;
        /** true: predict with this controller's acceleration estimate (low-passed change of the filtered velocity between
         *  solves, clamped to accelMaxIps2) instead of the acceleration passed to solve(). false = Python parity. */
        public boolean estimateAccel = false;
        public double accelTauS = 0.10;
        public double accelMaxIps2 = 300.0;
        // fire gate (FireGate; defaults = fire_gate.py constants)
        /** Movement protection (the settle gate) ON; false = fire whenever the solve is feasible. Default ON. */
        public boolean gateProtect = true;
        public double gateTurRelTolDeg = FireGate.TUR_REL_TOL_DEG;
        public double gateTurErrDeg = FireGate.TUR_ERR_DEG;
        public double gateTurRateCapDps = FireGate.TUR_RATE_CAP_DPS;
        public double gateTargetGain = FireGate.TUR_TARGET_GAIN;
        public double gateTReleaseS = FireGate.T_RELEASE_S;
        public double gateRpmTolRpm = FireGate.RPM_TOL;
        public double gateRpmStaticRps = FireGate.RPM_STATIC_RPS;
        public double gateOmegaMaxDps = FireGate.OMEGA_MAX_DPS;
        /** Hold the shot while |a| or |d speed/dt| of the filtered chassis velocity exceeds this [in/s^2]; 0 = off (practice bench: +2 % strict makes). */
        public double gateAccelHoldIps2 = 100.0;
        public double gateLookEps = FireGate.LOOK_EPS;
        /** false: skip the 5 look-ahead solves (p_ahead = 0, so never WAIT_BETTER). */
        public boolean gateLookAhead = true;
        /** Driver window after {@link #requestShot} [s] (gate bench: 1.0; the gate rarely waits > 0.15 s). */
        public double gateWindowS = 1.0;
        // bursts
        public boolean detectShotsFromRpm = false;
        public double detectDropFraction = 0.5;
        public double detectMinDropRpm = 40.0;
        public double burstResetS = 0.8;
    }

    /** Gate states: FireGate decisions 0..3, plus "no model". Index into {@link #GATE_TEXT}. */
    public static final int GATE_FIRE = FireGate.FIRE, GATE_INFEASIBLE = FireGate.WAIT_INFEASIBLE,
            GATE_UNSETTLED = FireGate.WAIT_UNSETTLED, GATE_BETTER = FireGate.WAIT_BETTER, GATE_NO_MODEL = 4;
    /** Old name of {@link #GATE_FIRE}. */
    public static final int GATE_OPEN = GATE_FIRE;
    public static final String[] GATE_TEXT = {"FIRE", "WAIT infeasible", "WAIT unsettled", "WAIT better ahead", "no model"};
    /** FireGate.refuseReason text (index = reason code). */
    public static final String[] REFUSE_TEXT = {"settled", "turret error", "turret error at release", "wheel rpm",
            "yaw rate", "turret speed"};
    /** A command-rate finite difference over a longer gap than this is treated as 0 (first solve after a pause). */
    public static final double CMD_RATE_MAX_GAP_S = 0.5;

    public final Params params = new Params();
    public final ReducedTargeter.Result result = new ReducedTargeter.Result();
    public final FireGate gate = new FireGate();
    /** Look-ahead claims at FireGate.HORIZONS_S (0 when infeasible), from the last solve. */
    public final double[] pAhead = new double[FireGate.HORIZONS_S.length];
    private final ReducedTargeter.Result ahead = new ReducedTargeter.Result();
    /** Solve at the state mechLeadS ahead (rpm and hood commands come from it when mechLeadS > 0). */
    public final ReducedTargeter.Result lead = new ReducedTargeter.Result();
    private final ShotFrame frame = new ShotFrame();
    private final double[] piv = new double[2];
    private ReducedTargeter targeter;
    private double modelPivotFwd, modelPivotRight, modelRpmMax;

    // outputs
    public double rpmCmd, hoodDeg, turretJavaDeg, turretTeamDeg, pMake;
    public boolean feasible, gateOpen;
    public int gateState = GATE_NO_MODEL;
    /** FireGate refuse reason of the last evaluation (0 = settled or not evaluated) and best look-ahead claim. */
    public int gateRefuse;
    public double gateBestAhead;
    /** Predicted turret aiming error at ball pass-through of the last gate evaluation [deg] (FireGate.turretReleaseError). */
    public double gateEstRelDeg;
    /** Window state of the last gate evaluation. */
    public double gateTLeft;
    public boolean gateDeadline, requestActive;
    /** fire_gate inputs: command rates of the last two solves [deg/s], [rpm/s]. */
    public double turCmdRate, rpmCmdRate;
    /** Effective wheel ceiling used by the last solve [rpm]. */
    public double rpmMaxEff;
    /** Real wheel rpm the model expects at release (result.rpmAtRelease x rpmScale) and the rpmScale of the last solve. */
    public double rpmAtReleaseReal, rpmScaleUsed = 1.0;
    /** Model rpm of the command (before rpmScale) and the team-frame acceleration used by the last solve [in/s^2]. */
    public double rpmModelCmd, accelX, accelY;
    private double accSolvePrevS = Double.NaN, accSolvePrevVx, accSolvePrevVy, estAx, estAy;
    public long solveNanos, maxSolveNanos, lookNanos, gateNanos;
    public double teamX, teamY, teamHeading, teamVx, teamVy;

    // burst bookkeeping
    public int shotIndex = 0, shotsDetected = 0;
    private double lastShotS = Double.NEGATIVE_INFINITY;
    private boolean detectArmed = true;
    // gate bookkeeping
    private double prevSolveS = Double.NaN, prevTurretCmd, prevRpmCmd;
    private double requestStartS;
    // acceleration hold: low-passed |a| and speed rate of the (filtered) team-frame chassis velocity [in/s^2]
    private double accPrevS = Double.NaN, accPrevVx, accPrevVy, accPrevSpeed, accAx, accAy, accDs;
    /** Acceleration estimate used by the hold [in/s^2] (max of |a| and |d speed/dt|). */
    public double gateAccelIps2;
    private static final double ACC_TAU_S = 0.10;

    public ReducedShotController() {
    }

    public boolean hasModel() {
        return targeter != null;
    }

    public ReducedShotModel model() {
        return targeter == null ? null : targeter.model();
    }

    /** Install a model (load-time; allocation allowed). */
    public void setModel(ReducedShotModel m) {
        if (m == null) {
            targeter = null;
            return;
        }
        modelPivotFwd = m.pivotFwdIn;
        modelPivotRight = m.pivotRightIn;
        modelRpmMax = m.rpmMax;
        rpmMaxEff = m.rpmMax;
        targeter = new ReducedTargeter(m);
        maxSolveNanos = 0;
        prevSolveS = Double.NaN;
    }

    /** Call when a ball is released (feeder code, or the rpm-dip detector below). Ends an open request. */
    public void notifyShot(double nowS) {
        if (nowS - lastShotS > params.burstResetS) shotIndex = 0;
        shotIndex++;
        shotsDetected++;
        lastShotS = nowS;
        requestActive = false;
    }

    /** The driver asks for a shot: opens the gate window (t_left = gateWindowS - elapsed; deadline at 0). */
    public void requestShot(double nowS) {
        requestActive = true;
        requestStartS = nowS;
    }

    public void cancelRequest() {
        requestActive = false;
    }

    /** Clear the burst state (e.g. when targeting stops). */
    public void resetBurst() {
        shotIndex = 0;
        lastShotS = Double.NEGATIVE_INFINITY;
        detectArmed = true;
    }

    /** Clear burst, request and command-rate history (targeting stopped). */
    public void reset() {
        resetBurst();
        requestActive = false;
        prevSolveS = Double.NaN;
        accSolvePrevS = Double.NaN;
        turCmdRate = 0.0;
        rpmCmdRate = 0.0;
    }

    private static double wrap180(double a) {
        return a - 360.0 * Math.floor((a + 180.0) / 360.0);
    }

    /**
     * Every loop: burst bookkeeping (reset after {@code burstResetS}, optional rpm-dip shot detector) and the fire
     * gate ({@link FireGate}, fed as {@code gate_bench.gate_policy} feeds {@code fire_gate}) against the latest solve.
     *
     * @param nowS              monotonic time [s]
     * @param wheelRpm          measured (filtered) flywheel rpm
     * @param turretMeasJavaDeg measured turret angle in the planner convention (encoder - turretZeroOffsetDeg)
     * @param turretRateDps     measured turret rate [deg/s] (encoder)
     * @param omegaDps          heading rate [deg/s]
     */
    public void sense(double nowS, double wheelRpm, double turretMeasJavaDeg, double turretRateDps, double omegaDps) {
        final Params P = params;
        // burst: the wheel table covers spacings up to its last node; after burstResetS the wheel has settled
        if (nowS - lastShotS > P.burstResetS) shotIndex = 0;
        // rpm-dip shot detector (optional): a release pulls the wheel down by ~result.dropRpm
        if (P.detectShotsFromRpm && targeter != null && rpmCmd > 0) {
            double thr = Math.max(P.detectMinDropRpm, P.detectDropFraction * result.dropRpm * rpmScaleUsed);
            double expected = shotIndex == 0 ? rpmCmd : rpmAtReleaseReal;
            if (detectArmed && wheelRpm < expected - thr) {
                notifyShot(nowS);
                detectArmed = false;
            } else if (!detectArmed && (wheelRpm > expected - 0.5 * thr || nowS - lastShotS > 0.12)) {
                detectArmed = true;
            }
        }
        if (targeter == null) {
            gateState = GATE_NO_MODEL;
            gateRefuse = 0;
            gateOpen = false;
            return;
        }
        long t0 = System.nanoTime();
        // turret error wrapped to +-180 (the encoder angle is multi-turn); fire_gate sees |cmd - meas| = |err|
        double turretErr = wrap180(turretMeasJavaDeg - turretJavaDeg);
        lastTurretErr = turretErr;
        if (requestActive) {
            gateTLeft = P.gateWindowS - (nowS - requestStartS);
            gateDeadline = gateTLeft <= 0.0;
        } else {
            gateTLeft = P.gateWindowS;
            gateDeadline = false;
        }
        FireGate g = gate;
        g.protect = P.gateProtect;
        g.turRelTolDeg = P.gateTurRelTolDeg;
        g.turErrDeg = P.gateTurErrDeg;
        g.turRateCapDps = P.gateTurRateCapDps;
        g.targetGain = P.gateTargetGain;
        g.tReleaseS = P.gateTReleaseS;
        g.rpmTol = P.gateRpmTolRpm;
        g.rpmStaticRps = P.gateRpmStaticRps;
        g.omegaMaxDps = P.gateOmegaMaxDps;
        g.lookEps = P.gateLookEps;
        // gate_bench: p = clip(p_make, 0, 1) if feasible else 0
        double pNow = feasible ? Math.min(Math.max(pMake, 0.0), 1.0) : 0.0;
        // NOTE (kept faithful to fire_gate.py): for queued balls (shotIndex >= 1) rpmCmd is the pre-compensated
        // command the wheel is intentionally not at, so the flywheel rule may report WAIT unsettled there.
        gateState = g.decide(feasible, pNow, pAhead, gateTLeft, turretJavaDeg, turretJavaDeg + turretErr,
                turretRateDps, turCmdRate, rpmCmd, wheelRpm, rpmCmdRate, omegaDps, gateDeadline);
        gateRefuse = g.lastRefuse;
        gateBestAhead = g.lastBest;
        gateEstRelDeg = g.lastEstRel;
        // acceleration hold (gate_policies.steady): not while accelerating or braking; the protection button disables it too
        double sp = Math.hypot(teamVx, teamVy);
        if (Double.isNaN(accPrevS)) {
            accAx = accAy = accDs = 0.0;
        } else {
            double dtA = nowS - accPrevS;
            if (dtA > 1e-6 && dtA <= 0.2) {
                double k = 1.0 - Math.exp(-dtA / ACC_TAU_S);
                accAx += k * ((teamVx - accPrevVx) / dtA - accAx);
                accAy += k * ((teamVy - accPrevVy) / dtA - accAy);
                accDs += k * ((sp - accPrevSpeed) / dtA - accDs);
            } else if (dtA > 0.2) {
                accAx = accAy = accDs = 0.0;
            }
        }
        accPrevS = nowS;
        accPrevVx = teamVx;
        accPrevVy = teamVy;
        accPrevSpeed = sp;
        gateAccelIps2 = Math.max(Math.hypot(accAx, accAy), Math.abs(accDs));
        if (P.gateProtect && P.gateAccelHoldIps2 > 0.0 && feasible && gateState == GATE_FIRE
                && gateAccelIps2 > P.gateAccelHoldIps2) {
            gateState = GATE_UNSETTLED;
            gateRefuse = 0;
        }
        gateOpen = gateState == GATE_FIRE;
        gateNanos = System.nanoTime() - t0;
    }

    /** Turret error used by the last gate evaluation [deg]. */
    public double lastTurretErr;

    /**
     * Look-ahead of the fire gate (exactly {@code gate_bench.trace_window}): claim of the solve at the team state
     * propagated by each FireGate.HORIZONS_S[j] with constant acceleration and turn rate ({@code RobotMotion.at}),
     * same shot index / spacing; 0 when infeasible. No allocation ({@code scratch} is overwritten).
     */
    public static void lookAhead(ReducedTargeter t, ReducedTargeter.Result scratch, double x, double y, double headingDeg,
                                 double vx, double vy, double omegaDps, double ax, double ay, int shotIndex,
                                 double spacingS, double[] out) {
        final double[] H = FireGate.HORIZONS_S;
        for (int j = 0; j < H.length; j++) {
            double dt = H[j];
            t.solve(x + vx * dt + 0.5 * ax * dt * dt, y + vy * dt + 0.5 * ay * dt * dt, headingDeg + omegaDps * dt,
                    vx + ax * dt, vy + ay * dt, omegaDps, ax, ay, shotIndex, spacingS, scratch);
            out[j] = scratch.feasible ? scratch.pMake : 0.0;
        }
    }

    /**
     * Solve for the current state (call at the planner period the turret was tuned with; it is cheap enough for
     * every loop). Updates {@link #rpmCmd}, {@link #hoodDeg}, {@link #turretJavaDeg}, {@link #pMake},
     * {@link #feasible}; {@link #sense} evaluates the gate.
     *
     * @param xCm,yCm,headingDeg  Odometry X(), Y(), Heading()
     * @param vxMps,vyMps         filtered field velocity [m/s] (PlannerInputFilter); axMps2/ayMps2 acceleration
     * @param omegaDps            heading rate [deg/s], same sense as Heading()
     */
    public void solve(double nowS, double xCm, double yCm, double headingDeg, double vxMps, double vyMps,
                      double omegaDps, double axMps2, double ayMps2) {
        if (targeter == null) {
            feasible = false;
            pMake = 0.0;
            return;
        }
        final Params P = params;
        ReducedShotModel m = targeter.model();
        // turret axis: the aim table is relative to the pivot, so the robot's real pivot can be used directly
        if (P.pivotFromRobot) {
            ShotFrame.pivotFromJava(P.pivotForwardM, P.pivotSidewaysM, piv);
            m.pivotFwdIn = piv[0];
            m.pivotRightIn = piv[1];
        } else {
            m.pivotFwdIn = modelPivotFwd;
            m.pivotRightIn = modelPivotRight;
        }
        // battery ceiling (R4 7.1 item 5): the solve clamps the inverse and the burst command to it. The solve works in MODEL
        // rpm; the real wheel turns rpmScale x that, so the real ceiling is ceiling / rpmScale in model rpm.
        double s = P.rpmScale > 0.05 ? P.rpmScale : 1.0;
        rpmScaleUsed = s;
        m.rpmMax = P.rpmMaxCeiling > 0.0 ? Math.min(modelRpmMax, P.rpmMaxCeiling / s) : modelRpmMax;
        rpmMaxEff = m.rpmMax * s;
        targeter.angleOffsetDeg = P.angleOffsetDeg;
        if (nowS - lastShotS > P.burstResetS) shotIndex = 0;
        double spacing = shotIndex >= 1 ? (nowS - lastShotS) + m.fireS : 0.0;

        frame.toTeam(P.blueAlliance, xCm, yCm, headingDeg, vxMps, vyMps, omegaDps, axMps2, ayMps2);
        teamX = frame.x;
        teamY = frame.y;
        teamHeading = frame.headingDeg;
        teamVx = frame.vx;
        teamVy = frame.vy;
        double ax = frame.ax, ay = frame.ay;
        if (P.estimateAccel) {
            double dtA = nowS - accSolvePrevS;
            if (Double.isNaN(accSolvePrevS) || !(dtA > 1e-6) || dtA > CMD_RATE_MAX_GAP_S) {
                estAx = estAy = 0.0;
            } else {
                double k = 1.0 - Math.exp(-dtA / Math.max(1e-3, P.accelTauS));
                estAx += k * ((frame.vx - accSolvePrevVx) / dtA - estAx);
                estAy += k * ((frame.vy - accSolvePrevVy) / dtA - estAy);
            }
            accSolvePrevS = nowS;
            accSolvePrevVx = frame.vx;
            accSolvePrevVy = frame.vy;
            double mag = Math.hypot(estAx, estAy);
            double c = P.accelMaxIps2 > 0.0 && mag > P.accelMaxIps2 ? P.accelMaxIps2 / mag : 1.0;
            ax = estAx * c;
            ay = estAy * c;
        }
        accelX = ax;
        accelY = ay;
        targeter.speedTrimPct = P.speedTrimPct;
        long t0 = System.nanoTime();
        targeter.solve(frame.x, frame.y, frame.headingDeg, frame.vx, frame.vy, frame.omegaDps,
                ax, ay, shotIndex, spacing, result);
        ReducedTargeter.Result cmd = result;
        double L = P.mechLeadS;
        if (L > 0.0) {
            targeter.solve(frame.x + frame.vx * L + 0.5 * ax * L * L, frame.y + frame.vy * L + 0.5 * ay * L * L,
                    frame.headingDeg + frame.omegaDps * L, frame.vx + ax * L, frame.vy + ay * L, frame.omegaDps,
                    ax, ay, shotIndex, spacing > 0.0 ? spacing + L : 0.0, lead);
            cmd = lead;
        }
        long t1 = System.nanoTime();
        solveNanos = t1 - t0;
        // fire-gate look-ahead: the same solve at the pose propagated by each horizon (team frame)
        if (P.gateLookAhead) {
            lookAhead(targeter, ahead, frame.x, frame.y, frame.headingDeg, frame.vx, frame.vy, frame.omegaDps,
                    ax, ay, shotIndex, spacing, pAhead);
        } else {
            for (int j = 0; j < pAhead.length; j++) pAhead[j] = 0.0;
        }
        lookNanos = System.nanoTime() - t1;
        if (solveNanos + lookNanos > maxSolveNanos) maxSolveNanos = solveNanos + lookNanos;

        rpmModelCmd = cmd.rpmCmd;
        rpmCmd = cmd.rpmCmd * s;
        rpmAtReleaseReal = cmd.rpmAtRelease * s;
        hoodDeg = Math.min(Math.max(cmd.hoodDeg + P.hoodTrimDeg, m.shotHood[0]), m.shotHood[m.nH - 1]);
        turretTeamDeg = result.turretDeg - (P.useModelTurretLead ? 0.0 : result.turretLeadDeg);
        turretJavaDeg = ShotFrame.turretTeamToJava(turretTeamDeg) + P.turretTrimDeg;
        pMake = result.pMake;
        feasible = result.feasible;

        // fire_gate tur_cmd_rate / rpm_cmd_rate: (command - previous solve's command) / dt (turret wrapped)
        double gap = nowS - prevSolveS;
        if (Double.isNaN(prevSolveS) || !(gap > 0.0) || gap > CMD_RATE_MAX_GAP_S) {
            turCmdRate = 0.0;
            rpmCmdRate = 0.0;
        } else {
            turCmdRate = wrap180(turretJavaDeg - prevTurretCmd) / gap;
            rpmCmdRate = (rpmCmd - prevRpmCmd) / gap;
        }
        prevSolveS = nowS;
        prevTurretCmd = turretJavaDeg;
        prevRpmCmd = rpmCmd;
    }
}

