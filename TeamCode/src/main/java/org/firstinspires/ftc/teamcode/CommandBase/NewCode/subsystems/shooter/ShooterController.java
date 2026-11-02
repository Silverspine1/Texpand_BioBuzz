package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.ShooterTuning;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.ShotControlTuning;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Localisation.Odometry;
import org.firstinspires.ftc.teamcode.CommandBase.LoopProfiler;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_command_base.Hardware.MotorEx;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;


/**
 * Shooter subsystem: flywheel, under-shooter hood and turret, all commanded by the REDUCED shot controller (the model
 * file). The old Shotplanner (a different shooter: 72 mm wheel, 45-75 deg hood) is deleted; without a model the flywheel
 * stays off and the turret is not driven.
 */
public class ShooterController extends SubSystem {

    ElapsedTime loopTimer = new ElapsedTime();
    FlywheelController flywheel = new FlywheelController();
    PlannerInputFilter inputFilter = new PlannerInputFilter();
    public AxonEncoder encoder;
    Odometry OdoMetry;

    Servo turretServo1;
    Servo turretServo2;
    SetTurretAngle turretController;

    Servo hoodRaw;
    /** HOOD COMMAND [deg of hood travel: 0 = bottom (far shots) .. 6.3 = top (near shots)]. Set from the reduced solve in
     *  runReducedShot() and trackHood(); execute() turns it into a servo position and writes the servo EVERY loop. */
    public double hoodTargetDeg = 0.0;
    /** Hood servo position (0..1) written this loop, for the displays. */
    public double hoodServoPos = Double.NaN;
    /** Where the turret gains came from, for the displays. */
    public String turretTuneSource = "not set";
    private double lastTurretCmd = Double.NaN;

    public MotorEx shooterMotor = new MotorEx();
    double targetRPM = 0;
    public double RPM;
    public double flywheelPower = 0;
    public double manualFlywheelPower = 0;
    public double flywheelTicksPerSecond = 0;
    /** Direction applied to the flywheel motor, and: power > 0.3 but the measured rpm negative (wrong motor direction or encoder sign). */
    public boolean appliedReversed = false, flywheelBackwards = false;
    public boolean targeting = false;
    private boolean ranTargeting;

    public enum TurretDebugMode { OFF, MANUAL, CLOSED_LOOP }
    public enum ServoSelect { BOTH, SERVO1_ONLY, SERVO2_ONLY }

    public TurretDebugMode turretDebugMode = TurretDebugMode.OFF;
    private TurretDebugMode lastTurretDebugMode = TurretDebugMode.OFF;
    public ServoSelect manualServoSelect = ServoSelect.BOTH;
    public double manualTurretPower = 0;

    public double plannerMs = 0;

    // ---- under-shooter REDUCED shot control: the only command source ----
    public final ReducedShotController reduced = new ReducedShotController();
    public String reducedStatus = "not loaded";
    /** True while the reduced controller is commanding rpm / hood / turret this loop. */
    public boolean reducedActive = false;
    /** ShotTuner only: > 0 replaces the reduced rpm command, finite replaces the hood command (the campaign's slip shot). */
    public double forceRpm = 0.0, forceHoodDeg = Double.NaN;
    private final ElapsedTime reducedClock = new ElapsedTime();
    private final ElapsedTime reducedSolveTimer = new ElapsedTime();
    private boolean reducedWasActive = false;
    /** Battery voltage for the reduced controller's rpm ceiling (read every 0.5 s; 0 = unknown). */
    public double batteryVolts = 0.0;
    private VoltageSensor battery;
    private final ElapsedTime batteryTimer = new ElapsedTime();

    public SetTurretAngle getTurretController() {
        return turretController;
    }

    public void testTurretTo(double angleDeg) {
        targeting = false;
        turretDebugMode = TurretDebugMode.CLOSED_LOOP;
        turretController.setTarget(angleDeg, 1.0, false);
    }

    public ShooterController(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, returnDefaultCommand());
        this.OdoMetry = opModeEX.odometry;
    }

    @Override
    public void init() {
        ShooterTuning.load();
        ShotControlTuning.hoodDebug = false;

        shooterMotor.initMotor("shooterMotor", getOpMode().hardwareMap);

        turretServo1 = getOpMode().hardwareMap.get(Servo.class, "turretServo1");
        turretServo2 = getOpMode().hardwareMap.get(Servo.class, "turretServo2");

        encoder = new AxonEncoder();
        encoder.init(getOpMode().hardwareMap, "turretEncoder");
        hoodRaw = getOpMode().hardwareMap.get(Servo.class, "hoodServ");

        turretController = new SetTurretAngle(
                turretServo1,
                turretServo2,
                encoder,
                220.0,    // max turret velocity, deg/s
                1200.0    // max turret acceleration, deg/s²
        );

        applyTuning();
        loopTimer.reset();
        zeroTurretAtStart();

        ShotControlTuning.load();
        applyMotorDirection();
        applyTurretTune();
        try {
            battery = getOpMode().hardwareMap.voltageSensor.iterator().next();
            batteryVolts = battery.getVoltage();
        } catch (Throwable t) {
            battery = null;          // no ceiling from the battery (model rpm_max)
        }
        loadReducedModel();
    }

    /**
     * Turret gains: the saved turret_tune.properties if it loads, otherwise the controller fitted to this robot's own
     * 2026-10-30 tuner log (predictive mode; the bare constructor defaults oscillate ~6 deg peak-to-peak in the rig).
     * ShotControlTuning.turretKPos then softens the position gain.
     */
    private void applyTurretTune() {
        TurretTune t = TurretTune.load();
        if (t != null) {
            turretTuneSource = "saved turret_tune.properties";
        } else {
            t = new TurretTune();
            t.kS = 0.0284; t.kV = 0.0005; t.kA = 0.0; t.kP = 0.001; t.kVelocityFeedback = 0.0;
            t.maxCorrection = 0.14; t.maxVelocity = 236; t.maxAcceleration = 2200; t.latency = 0.0;
            t.brakeAware = true; t.brakeDelayScale = 1.0;
            t.plantDelay = 0.045; t.plantAccel = 1500; t.plantDecel = 1500; t.kPos = 15; t.decelSafety = 0.8;
            t.ffSpeed = new double[]{0, 103, 150, 228, 315};
            t.ffPower = new double[]{0.0284, 0.0428, 0.0568, 0.0801, 0.1174};
            turretTuneSource = "BUILT-IN fit (no valid turret_tune.properties on the hub)";
        }
        if (ShotControlTuning.turretKPos > 0) t.kPos = ShotControlTuning.turretKPos;
        turretTuneSource += ", kPos " + t.kPos + (t.brakeAware ? ", predictive" : ", profile mode");
        t.applyTo(turretController);
    }

    /**
     * Loads the model {@code ShotControlTuning.modelFile}: from /sdcard/FIRST/settings/ when a file of that name was pushed
     * there (override), otherwise the copy BUILT INTO the app (TeamCode/src/main/assets/), so a fresh hub works without adb.
     * Then the optional field-tune export. Never throws: a bad file leaves no model (flywheel off, status says why).
     */
    /** The model file bundled in TeamCode/src/main/assets/ (fallback when ShotControlTuning.modelFile is not found). */
    public static final String BUILT_IN_MODEL = "robot_model_red_audience_r4h.json";

    /** One short line for the top of every display: model, targeting, range, hood command and the servo position written. */
    public String hoodStatusLine() {
        String check = ShotControlTuning.hoodDebug ? "  [HOOD DEBUG: servo set by hand, gamepad2 Y = off]" : "";
        if (!reduced.hasModel()) {
            return String.format(java.util.Locale.US, "NO MODEL (%s) -> hood held at bottom, servo %.3f%s", reducedStatus, hoodServoPos, check);
        }
        return String.format(java.util.Locale.US, "range %.0f in -> hood %.1f of 6.3 deg -> servo %.3f (bottom %.3f, top %.3f)  targeting %s%s",
                reduced.result.d, hoodTargetDeg, hoodServoPos,
                ShotControlTuning.hoodBottomPos, ShotControlTuning.hoodTopPos, targeting ? "ON" : "OFF", check);
    }

    public void loadReducedModel() {
        try {
            String name = ShotControlTuning.modelFile == null || ShotControlTuning.modelFile.trim().isEmpty()
                    ? ReducedShotModel.MODEL_FILE : ShotControlTuning.modelFile.trim();
            java.io.File f = ShotControlTuning.settingsFile(name);
            ReducedShotModel m;
            String source;
            if (f.exists()) {
                m = ReducedShotModel.load(f);
                source = f.getPath();
            } else {
                java.io.InputStream in;
                try {
                    in = getOpMode().hardwareMap.appContext.getAssets().open(name);
                } catch (java.io.IOException e) {
                    try {                       // e.g. an old file name restored from shot_control.properties
                        in = getOpMode().hardwareMap.appContext.getAssets().open(BUILT_IN_MODEL);
                        name = BUILT_IN_MODEL + " (" + ShotControlTuning.modelFile + " not found)";
                    } catch (java.io.IOException e2) {
                        reduced.setModel(null);
                        reducedStatus = "missing: " + name + " is neither in " + f.getParent() + " nor built into the app";
                        return;
                    }
                }
                try {
                    m = ReducedShotModel.fromJson(ReducedShotModel.readText(in));
                } finally {
                    in.close();
                }
                source = "built-in " + name;
            }
            java.io.File ft = ShotControlTuning.settingsFile(ShotControlTuning.fieldTuneFile == null
                    || ShotControlTuning.fieldTuneFile.trim().isEmpty()
                    ? ReducedShotModel.FIELD_TUNE_FILE : ShotControlTuning.fieldTuneFile.trim());
            if (ShotControlTuning.useFieldTune && ft.exists()) {
                m.applyFieldTune(ReducedShotModel.readText(ft));
            }
            reduced.setModel(m);
            reducedStatus = "loaded (" + source + "): " + m.describe();
        } catch (Throwable t) {
            reduced.setModel(null);
            reducedStatus = "load failed: " + t.getMessage();
        }
    }

    /**
     * The turret is ASSUMED to point exactly straight forward when the program starts (ShooterTuning.assumeTurretForwardAtStart,
     * default true): zero the encoder where it is and ignore any saved turretZeroOffsetDeg. Called at init() and again at PLAY
     * (OpModeEX.start), so a turret nudged by hand during INIT is still taken as forward.
     */
    public void zeroTurretAtStart() {
        if (!ShooterTuning.assumeTurretForwardAtStart || encoder == null) return;
        encoder.zeroHere();
        ShooterTuning.turretZeroOffsetDeg = 0.0;
        if (turretController != null) turretController.stop();
    }

    /** Applies ShotControlTuning.shooterMotorReversed to the flywheel motor (init, and live when it changes). */
    private void applyMotorDirection() {
        appliedReversed = ShotControlTuning.shooterMotorReversed;
        shooterMotor.setDirection(appliedReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    private void applyTuning() {
        flywheel.kF = ShooterTuning.flywheelKF;
        flywheel.kP = ShooterTuning.flywheelKP;
        flywheel.kI = ShooterTuning.flywheelKI;
        flywheel.integralMax = ShooterTuning.flywheelIntegralMax;
        flywheel.integralZoneRpm = ShooterTuning.flywheelIntegralZoneRpm;
        flywheel.maxPower = Math.max(0, Math.min(1, ShooterTuning.flywheelMaxPower));

        inputFilter.timeConstantSeconds = ShooterTuning.plannerVelocityFilterSeconds;
        inputFilter.velocityDeadbandMps = ShooterTuning.plannerVelocityDeadbandMps;
        inputFilter.headingRateDeadbandDps = ShooterTuning.plannerHeadingRateDeadbandDps;
        inputFilter.turretRateDeadbandDps = ShooterTuning.plannerTurretRateDeadbandDps;
        inputFilter.useAcceleration = ShooterTuning.plannerUseAcceleration;
    }

    /** Where the turret is actually pointing, robot-relative (encoder reading minus the zero offset). */
    public double getTurretAimDeg() {
        return encoder.getTurretAngle() - ShooterTuning.turretZeroOffsetDeg;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public boolean flywheelReady() {
        return targetRPM > 0
                && Math.abs(RPM - targetRPM) <= targetRPM * ShooterTuning.flywheelReadyPercent / 100.0;
    }

    @Override
    public void execute() {
        long start = System.nanoTime();
        ranTargeting = false;
        applyTuning();
        encoder.UpdatePosition();

        double dt = Math.min(0.1, loopTimer.seconds());
        loopTimer.reset();

        inputFilter.update(
                OdoMetry.getXVelocity() / 100,
                OdoMetry.getYVelocity() / 100,
                OdoMetry.getHeadingVelocityDeg(),
                encoder.getVelocity(),
                OdoMetry.getXAcceleration() / 100,
                OdoMetry.getYAcceleration() / 100,
                OdoMetry.getHeadingAccelerationDeg(),
                dt
        );

        flywheelTicksPerSecond = shooterMotor.getVelocity();
        double rawRpm = flywheelTicksPerSecond / Math.max(1, ShooterTuning.flywheelTicksPerRev) * 60;
        double filter = Math.max(0.01, Math.min(1, ShooterTuning.flywheelRpmFilter));
        RPM += filter * (rawRpm - RPM);
        if (ShotControlTuning.shooterMotorReversed != appliedReversed) applyMotorDirection();
        flywheelBackwards = flywheelPower > 0.3 && RPM < -300;

        if (targeting && turretDebugMode == TurretDebugMode.CLOSED_LOOP) {
            turretDebugMode = TurretDebugMode.OFF;
        }

        if (turretDebugMode == TurretDebugMode.MANUAL) {
            if (lastTurretDebugMode != TurretDebugMode.MANUAL) {
                turretController.stop();
            }
            double cmd = Math.max(0, Math.min(1, 0.5 + manualTurretPower));
            turretServo1.setPosition(manualServoSelect == ServoSelect.SERVO2_ONLY ? 0.5 : cmd);
            turretServo2.setPosition(manualServoSelect == ServoSelect.SERVO1_ONLY ? 0.5 : cmd);
            stopFlywheel();

        } else if (turretDebugMode == TurretDebugMode.CLOSED_LOOP) {
            stopFlywheel();

        } else if (targeting) {
            ranTargeting = true;
            reducedActive = reduced.hasModel();
            if (reducedActive) {
                runReducedShot();
            } else {                       // no model: nothing to aim with; wheel off, turret not driven (status says why)
                targetRPM = 0;
                turretController.stop();
            }
            reducedWasActive = reducedActive;

            flywheelPower = flywheel.update(targetRPM, RPM, dt);
            shooterMotor.update(flywheelPower);

        } else {
            stopFlywheel();
            turretController.stop();
            lastTurretCmd = Double.NaN;
            reduced.reset();
            reducedActive = false;
            reducedWasActive = false;
        }

        // ---- HOOD: hoodTargetDeg (from the shot solve) -> servo position -> servo, EVERY loop, in every mode ----
        if (!ranTargeting) trackHood();                // targeting off / turret test modes: the hood still follows the range
        double pos = underHoodServoPos(hoodTargetDeg);
        if (ShotControlTuning.hoodDebug) pos = ShotControlTuning.hoodDebugPos;          // gamepad2 Y: HOOD DEBUG (trimButtons)
        if (!Double.isNaN(pos)) {                       // NaN pose -> NaN solve: keep the last position
            hoodServoPos = pos;
            hoodRaw.setPosition(pos);
        }

        lastTurretDebugMode = turretDebugMode;

        if (turretDebugMode != TurretDebugMode.MANUAL) {
            turretController.update();
        }

        ((OpModeEX) getOpMode()).profiler.recordDuration(LoopProfiler.TURRET, System.nanoTime() - start);
    }

    private void stopFlywheel() {
        targetRPM = 0;
        flywheelPower = Math.max(0, Math.min(1, manualFlywheelPower));
        flywheel.reset();
        shooterMotor.update(flywheelPower);
    }


    /**
     * REDUCED targeter path: the same turret controller (SetTurretAngle) and flywheel loop as before, with the rpm,
     * hood and turret set-points from the velocity-compensated aim table. Solves at ShotControlTuning.solvePeriodMs
     * (the turret's tuned command period); the fire gate is evaluated every loop.
     */
    private void applyReducedParams() {
        ReducedShotController.Params p = reduced.params;
        p.blueAlliance = ShotControlTuning.blueAlliance;
        p.pivotFromRobot = ShotControlTuning.pivotFromShooterTuning;
        p.pivotForwardM = ShooterTuning.turretAxisForwardMeters;
        p.pivotSidewaysM = ShooterTuning.turretAxisSidewaysMeters;
        p.useModelTurretLead = ShotControlTuning.useModelTurretLead;
        p.rpmScale = ShotControlTuning.rpmScale;
        p.angleOffsetDeg = ShotControlTuning.angleOffsetDeg;
        p.globalGoal = true;
        p.goalTrimXcm = ShotControlTuning.goalTrimXcm;
        p.goalTrimYcm = ShotControlTuning.goalTrimYcm;
        p.turretScale = ShotControlTuning.turretScale > 0.5 ? ShotControlTuning.turretScale : 1.0;
        p.turretTrimDeg = ShotControlTuning.turretTrimDeg;
        p.gateProtect = ShotControlTuning.gateProtect;
        p.gateTurRelTolDeg = ShotControlTuning.gateTurRelTolDeg;
        p.gateTurErrDeg = ShotControlTuning.gateTurErrDeg;
        p.gateTurRateCapDps = ShotControlTuning.gateTurRateCapDps;
        p.gateTargetGain = ShotControlTuning.gateTargetGain;
        p.gateTReleaseS = ShotControlTuning.gateTReleaseS;
        p.gateRpmTolRpm = ShotControlTuning.gateRpmTolRpm;
        p.gateRpmStaticRps = ShotControlTuning.gateRpmStaticRps;
        p.gateOmegaMaxDps = ShotControlTuning.gateOmegaMaxDps;
        p.gateAccelHoldIps2 = ShotControlTuning.gateAccelHoldIps2;
        p.mechLeadS = ShotControlTuning.mechDelayS;
        p.estimateAccel = ShotControlTuning.useAcceleration;
        p.accelTauS = ShotControlTuning.accelFilterS;
        p.accelMaxIps2 = ShotControlTuning.accelMaxIps2;
        p.gateLookEps = ShotControlTuning.gateLookEps;
        p.gateLookAhead = ShotControlTuning.gateLookAhead;
        p.gateWindowS = ShotControlTuning.gateWindowS;
        // battery wheel ceiling (R4): fraction x no-load x V/12; reads are slow, so every 0.5 s
        if (battery != null && batteryTimer.seconds() > 0.5) {
            batteryTimer.reset();
            try {
                batteryVolts = battery.getVoltage();
            } catch (Throwable t) {
                batteryVolts = 0.0;
            }
        }
        p.rpmMaxCeiling = ShotControlTuning.rpmMaxFromBattery && batteryVolts > 6.0
                ? ShotControlTuning.rpmMaxNoLoadFraction * ShotControlTuning.wheelNoLoadRpmAt12V * batteryVolts / 12.0
                : 0.0;
        p.detectShotsFromRpm = ShotControlTuning.detectShotsFromRpm;
        p.detectDropFraction = ShotControlTuning.detectDropFraction;
        p.detectMinDropRpm = ShotControlTuning.detectMinDropRpm;
        p.burstResetS = ShotControlTuning.burstResetS;
    }

    private void solveReduced(double now) {
        reduced.solve(now,
                OdoMetry.X(), OdoMetry.Y(), OdoMetry.Heading(),
                inputFilter.velocityX(), inputFilter.velocityY(),
                inputFilter.headingRate(),
                inputFilter.accelerationX(), inputFilter.accelerationY());
        plannerMs = reduced.solveNanos / 1e6;
    }

    /** Targeting OFF: the hood still follows the range (solve only; wheel and turret untouched). */
    private void trackHood() {
        if (!reduced.hasModel() || reducedSolveTimer.milliseconds() < ShotControlTuning.solvePeriodMs) return;
        reducedSolveTimer.reset();
        applyReducedParams();
        solveReduced(reducedClock.seconds());
        hoodTargetDeg = reduced.hoodDeg;
    }

    private void runReducedShot() {
        applyReducedParams();
        double now = reducedClock.seconds();
        if (!reducedWasActive || reducedSolveTimer.milliseconds() >= ShotControlTuning.solvePeriodMs) {
            reducedSolveTimer.reset();
            solveReduced(now);
            // infeasible states still get a defined tracking command (nearest feasible node), so always apply it
            targetRPM = reduced.rpmCmd;
            if (forceRpm > 0.0) targetRPM = forceRpm;                       // ShotTuner slip-check shot (5700 rpm, hood 0)
            double turCmd = reduced.turretJavaDeg + ShooterTuning.turretZeroOffsetDeg;
            boolean still = Math.hypot(inputFilter.velocityX(), inputFilter.velocityY()) * 39.37 < ShotControlTuning.turretHoldMaxSpeedIps
                    && Math.abs(inputFilter.headingRate()) < ShotControlTuning.turretHoldMaxYawDps;
            double step = Double.isNaN(lastTurretCmd) ? 1e9 : Math.abs(((turCmd - lastTurretCmd + 540.0) % 360.0) - 180.0);
            if (still && step <= ShotControlTuning.turretHoldBandDeg) turCmd = lastTurretCmd;   // hold: ignore sub-band noise
            lastTurretCmd = turCmd;
            turretController.setTarget(turCmd, 1.0, false);
            hoodTargetDeg = Double.isNaN(forceHoodDeg) ? reduced.hoodDeg : forceHoodDeg;
        }
        reduced.sense(now, RPM, getTurretAimDeg(), encoder.getVelocity(), OdoMetry.getHeadingVelocityDeg());
    }

    /** Driver asks for a shot: opens the reduced fire gate's window (the deadline fires a feasible solve; see ShotHold). */
    public void requestReducedShot() {
        reduced.requestShot(reducedClock.seconds());
    }

    /** The feed started (a ball is on its way): burst bookkeeping of the reduced controller, ends the request window. */
    public void notifyReducedShot() {
        reduced.notifyShot(reducedClock.seconds());
    }

    /** Seconds on the reduced controller's clock (the same one requestReducedShot / sense use). */
    public double reducedSeconds() {
        return reducedClock.seconds();
    }

    /** Hood travel (0..6.3 deg above the lower stop) -> servo position 0..1 (ShotControlTuning.hoodBottomPos / hoodTopPos). */
    public static double underHoodServoPos(double hoodTravelDeg) {
        double f = Math.max(0.0, Math.min(6.3, hoodTravelDeg)) / 6.3;
        return ShotControlTuning.hoodBottomPos + f * (ShotControlTuning.hoodTopPos - ShotControlTuning.hoodBottomPos);
    }

    /** Fire gate of the reduced controller (R4/R5 FireGate decision FIRE; false whenever it is not the active command
     *  source). Nothing in this class releases a ball: {@link ShotHold} turns it into a feed permission. */
    public boolean shotGateOpen() {
        return reducedActive && reduced.gateOpen;
    }
}
