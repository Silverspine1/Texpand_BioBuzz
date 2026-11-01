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
import dev.weaponboy.nexus_command_base.Hardware.ServoDegrees;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;


public class ShooterController extends SubSystem {

    Shotplanner shotPlanner = new Shotplanner(new Shotplanner.Config());
    ElapsedTime plannerTimer = new ElapsedTime();
    ElapsedTime loopTimer = new ElapsedTime();
    FlywheelController flywheel = new FlywheelController();
    PlannerInputFilter inputFilter = new PlannerInputFilter();
    ShotSolutionHold solutionHold = new ShotSolutionHold();
    boolean wasTargeting = false;
    public AxonEncoder encoder;
    Odometry OdoMetry;

    public Shotplanner.BlueHiveSide blueHiveSide = Shotplanner.BlueHiveSide.AUDIENCE;
    public ServoDegrees Hood = new ServoDegrees();
    Servo turretServo1;
    Servo turretServo2;
    SetTurretAngle turretController;

    Servo hoodRaw;
    /** Last hood servo position written by the reduced path (0..1), for the displays. */
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

    public enum TurretDebugMode { OFF, MANUAL, CLOSED_LOOP }
    public enum ServoSelect { BOTH, SERVO1_ONLY, SERVO2_ONLY }

    public TurretDebugMode turretDebugMode = TurretDebugMode.OFF;
    private TurretDebugMode lastTurretDebugMode = TurretDebugMode.OFF;
    public ServoSelect manualServoSelect = ServoSelect.BOTH;
    public double manualTurretPower = 0;

    public Shotplanner.ShotSolution lastShot;
    public Shotplanner.RobotState lastRobotState;
    public double plannerMs = 0;

    // ---- under-shooter REDUCED shot control (ShotControlTuning.enabled; OFF = old planner path unchanged) ----
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

        shooterMotor.initMotor("shooterMotor", getOpMode().hardwareMap);

        Hood.initServo("hoodServ",getOpMode().hardwareMap);
        turretServo1 = getOpMode().hardwareMap.get(Servo.class, "turretServo1");
        turretServo2 = getOpMode().hardwareMap.get(Servo.class, "turretServo2");

        encoder = new AxonEncoder();
        encoder.init(getOpMode().hardwareMap, "turretEncoder");
        Hood.setRange(355);
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
        if (reduced.hasModel()) ShotControlTuning.enabled = true;   // the old planner is for a different shooter
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
     * Loads /sdcard/FIRST/settings/{ShotControlTuning.modelFile} (default robot_model_red_audience.json; recommended
     * robot_model_red_audience_r4.json) + the optional field-tune export. Never throws: a missing or bad file leaves
     * the reduced controller without a model and the old planner in charge.
     */
    public void loadReducedModel() {
        try {
            String name = ShotControlTuning.modelFile == null || ShotControlTuning.modelFile.trim().isEmpty()
                    ? ReducedShotModel.MODEL_FILE : ShotControlTuning.modelFile.trim();
            java.io.File f = ShotControlTuning.settingsFile(name);
            if (!f.exists()) {
                reduced.setModel(null);
                reducedStatus = "missing " + f.getPath();
                return;
            }
            ReducedShotModel m = ReducedShotModel.load(f);
            java.io.File ft = ShotControlTuning.settingsFile(ShotControlTuning.fieldTuneFile == null
                    || ShotControlTuning.fieldTuneFile.trim().isEmpty()
                    ? ReducedShotModel.FIELD_TUNE_FILE : ShotControlTuning.fieldTuneFile.trim());
            if (ShotControlTuning.useFieldTune && ft.exists()) {
                m.applyFieldTune(ReducedShotModel.readText(ft));
            }
            reduced.setModel(m);
            reducedStatus = "loaded: " + m.describe();
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
        Shotplanner.Config c = shotPlanner.getConfig();
        c.flywheelRadiusMeters = Math.max(0.001, ShooterTuning.flywheelRadiusMeters);
        c.muzzleEfficiency = Math.max(0.05, ShooterTuning.muzzleEfficiency);
        c.magnusK = ShooterTuning.magnusK;
        c.maxFlywheelRPM = Math.max(100, ShooterTuning.maxFlywheelRPM);
        c.requiredClearanceMeters = ShooterTuning.requiredClearanceMeters;
        c.muzzleHeightMeters = ShooterTuning.muzzleHeightMeters;
        c.turretAxisForwardMeters = ShooterTuning.turretAxisForwardMeters;
        c.muzzleForwardFromTurretMeters = ShooterTuning.muzzleForwardFromTurretMeters;
        c.turretAxisLeftMeters = ShooterTuning.turretAxisSidewaysMeters;

        double low = Math.min(ShooterTuning.hoodLaunchAtMinDeg, ShooterTuning.hoodLaunchAtMaxDeg);
        double high = Math.max(ShooterTuning.hoodLaunchAtMinDeg, ShooterTuning.hoodLaunchAtMaxDeg);
        c.minLaunchAngleDeg = low;
        c.maxLaunchAngleDeg = Math.max(high, low + 1);

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
        solutionHold.holdSolves = (int) Math.max(0, ShooterTuning.plannerHoldSolves);
    }

    public void setHoodDegrees(double theta) {
        double launchSpan = ShooterTuning.hoodLaunchAtMaxDeg - ShooterTuning.hoodLaunchAtMinDeg;
        double servoSpan = ShooterTuning.hoodServoAtMaxDeg - ShooterTuning.hoodServoAtMinDeg;
        double fraction = Math.abs(launchSpan) < 1e-6 ? 0 : (theta - ShooterTuning.hoodLaunchAtMinDeg) / launchSpan;
        double servoPos = ShooterTuning.hoodServoAtMinDeg + fraction * servoSpan;

        double lowLimit = Math.min(ShooterTuning.hoodServoAtMinDeg, ShooterTuning.hoodServoAtMaxDeg);
        double highLimit = Math.max(ShooterTuning.hoodServoAtMinDeg, ShooterTuning.hoodServoAtMaxDeg);
        Hood.setPosition(Math.max(lowLimit, Math.min(highLimit, servoPos)));
    }

    public Shotplanner.HiveTarget getHiveTarget() {
        return getHiveTarget(blueHiveSide);
    }

    public static Shotplanner.HiveTarget getHiveTarget(Shotplanner.BlueHiveSide side) {
        boolean audience = side == Shotplanner.BlueHiveSide.AUDIENCE;
        return new Shotplanner.HiveTarget(
                audience ? ShooterTuning.audienceGoalX : ShooterTuning.oppositeGoalX,
                audience ? ShooterTuning.audienceGoalY : ShooterTuning.oppositeGoalY,
                ShooterTuning.openingBottomZ,
                audience ? 180.0 : 0.0
        );
    }

    public Shotplanner.Config getPlannerConfig() {
        return shotPlanner.getConfig();
    }

    /** Turret angle that points straight at the goal from the turret axis, in the planner's convention. */
    public double getGoalBearingDeg() {
        Shotplanner.RobotState pose = new Shotplanner.RobotState(
                OdoMetry.X() / 100, OdoMetry.Y() / 100, 0, 0, OdoMetry.Heading(), 0, 0);
        double bearing = shotPlanner.bearingToTargetDeg(pose, getHiveTarget()) - OdoMetry.Heading();
        bearing %= 360.0;
        if (bearing > 180) bearing -= 360;
        if (bearing <= -180) bearing += 360;
        return bearing;
    }

    /** Where the turret is actually pointing, in the planner's convention (encoder reading minus the zero offset). */
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
            reducedActive = ShotControlTuning.enabled && reduced.hasModel();
            if (reducedActive) {
                runReducedShot();
            } else if (!wasTargeting || plannerTimer.milliseconds() >= ShooterTuning.plannerPeriodMs) {
                plannerTimer.reset();
                runShotPlanner();
            }
            reducedWasActive = reducedActive;

            flywheelPower = flywheel.update(targetRPM, RPM, dt);
            shooterMotor.update(flywheelPower);

        } else {
            stopFlywheel();
            turretController.stop();
            lastTurretCmd = Double.NaN;
            solutionHold.reset();
            reduced.reset();
            reducedActive = false;
            reducedWasActive = false;
        }

        wasTargeting = targeting && turretDebugMode == TurretDebugMode.OFF;
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

    private void runShotPlanner() {
        Shotplanner.RobotState robot = new Shotplanner.RobotState(
                OdoMetry.X()/100,
                OdoMetry.Y()/100,

                inputFilter.velocityX(),
                inputFilter.velocityY(),

                inputFilter.accelerationX(),
                inputFilter.accelerationY(),

                OdoMetry.Heading(),
                inputFilter.headingRate(),
                inputFilter.headingAcceleration(),

                ShooterTuning.plannerUseTurretRate ? inputFilter.turretRate() : 0
        );

        long plannerStart = System.nanoTime();
        Shotplanner.ShotSolution shot = shotPlanner.calculateShot(
                robot,
                getHiveTarget(),
                ShooterTuning.plannerDelaySeconds
        );
        plannerMs = (System.nanoTime() - plannerStart) / 1e6;
        lastShot = shot;
        lastRobotState = robot;

        Shotplanner.ShotSolution act = solutionHold.apply(shot, OdoMetry.Heading());

        if (act.reachable
                && Double.isFinite(act.turretAngleDeg)
                && Double.isFinite(act.launchAngleDeg)
                && Double.isFinite(act.flywheelRPM)) {
            targetRPM = act.flywheelRPM;
            turretController.setTarget(act.turretAngleDeg + ShooterTuning.turretZeroOffsetDeg, 1.0, false);
            setHoodDegrees(act.launchAngleDeg);
        } else {
            turretController.setTarget(getGoalBearingDeg() + ShooterTuning.turretZeroOffsetDeg, 1.0, false);
        }
    }

    /**
     * REDUCED targeter path: the same turret controller (SetTurretAngle) and flywheel loop as before, with the rpm,
     * hood and turret set-points from the velocity-compensated aim table. Solves at ShotControlTuning.solvePeriodMs
     * (the turret's tuned command period); the fire gate is evaluated every loop.
     */
    private void runReducedShot() {
        ReducedShotController.Params p = reduced.params;
        p.blueAlliance = ShotControlTuning.blueAlliance;
        p.pivotFromRobot = ShotControlTuning.pivotFromShooterTuning;
        p.pivotForwardM = ShooterTuning.turretAxisForwardMeters;
        p.pivotSidewaysM = ShooterTuning.turretAxisSidewaysMeters;
        p.useModelTurretLead = ShotControlTuning.useModelTurretLead;
        p.speedTrimPct = ShotControlTuning.speedTrimPct;
        p.turretTrimDeg = ShotControlTuning.turretTrimDeg;
        p.hoodTrimDeg = ShotControlTuning.hoodTrimDeg;
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

        double now = reducedClock.seconds();
        if (!reducedWasActive || reducedSolveTimer.milliseconds() >= ShotControlTuning.solvePeriodMs) {
            reducedSolveTimer.reset();
            reduced.solve(now,
                    OdoMetry.X(), OdoMetry.Y(), OdoMetry.Heading(),
                    inputFilter.velocityX(), inputFilter.velocityY(),
                    inputFilter.headingRate(),
                    inputFilter.accelerationX(), inputFilter.accelerationY());
            plannerMs = reduced.solveNanos / 1e6;
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
            setUnderHoodDegrees(Double.isNaN(forceHoodDeg) ? reduced.hoodDeg : forceHoodDeg);
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

    /** Under-shooter hood: travel 0..6.3 deg above the lower stop -> servo degrees (ShotControlTuning). */
    public void setUnderHoodDegrees(double hoodTravelDeg) {
        hoodServoPos = underHoodServoPos(hoodTravelDeg);
        hoodRaw.setPosition(hoodServoPos);
    }

    /** Hood travel (0..6.3 deg above the lower stop) -> servo position 0..1 (ShotControlTuning.hoodServoPosAtZero / AtMax). */
    public static double underHoodServoPos(double hoodTravelDeg) {
        double f = Math.max(0.0, Math.min(6.3, hoodTravelDeg)) / 6.3;
        return ShotControlTuning.hoodServoPosAtZero + f * (ShotControlTuning.hoodServoPosAtMax - ShotControlTuning.hoodServoPosAtZero);
    }

    /** Fire gate of the reduced controller (R4/R5 FireGate decision FIRE; false whenever it is not the active command
     *  source). Nothing in this class releases a ball: {@link ShotHold} turns it into a feed permission. */
    public boolean shotGateOpen() {
        return reducedActive && reduced.gateOpen;
    }
}
