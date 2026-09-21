package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Drivebase.DriveBase;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Intake.Transfer;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Intake.intakeHardware;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.ShooterController;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Localisation.Odometry;

import dev.weaponboy.nexus_command_base.OpmodeEX.Scheduler;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;

import java.util.List;

public abstract class OpModeEX extends OpMode {
    public intakeHardware IntakeHardware = new intakeHardware(this);

    public Odometry odometry = new Odometry(this);
    public Transfer transfer = new Transfer(this);
    public ShooterController shooterController = new ShooterController(this);
    public DriveBase driveBase = new DriveBase(this);





    private final Scheduler scheduler = new Scheduler(this,
            new SubSystem[] {odometry,transfer,IntakeHardware,shooterController,driveBase});

    List<LynxModule> allHubs;

    public ElapsedTime autoTime = new ElapsedTime();

    public ElapsedTime timer = new ElapsedTime();
    public ElapsedTime telemetryTimer = new ElapsedTime();
    double lastTime;
    public double loopTime;

    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad lastGamepad1 = new Gamepad();

    public Gamepad currentGamepad2 = new Gamepad();
    public Gamepad lastGamepad2 = new Gamepad();

    // ── Profiler ───────────────────────────────────────────────
    public final LoopProfiler profiler = new LoopProfiler();

    public abstract void initEX();

    public abstract void loopEX();

    @Override
    public void init() {

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Throttle telemetry transmission to 5 Hz (both explicit and SDK auto)
        telemetry.setMsTransmissionInterval(200);

        timer.reset();
        scheduler.init();
        initEX();
    }

    @Override
    public void start() {
        autoTime.reset();
        super.start();
    }

    @Override
    public void loop() {
        profiler.startTotalLoop(); // totalLoop start

        // ── Phase: bulkCacheClear ──
        profiler.startPhase();
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        profiler.endPhase(LoopProfiler.BULK_CACHE_CLEAR);

        // ── Phase: gamepadCopy ──
        profiler.startPhase();
        lastGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        lastGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
        profiler.endPhase(LoopProfiler.GAMEPAD_COPY);

        lastTime = timer.milliseconds();

        // ── Phase: schedulerExecute ──
        profiler.startPhase();
        scheduler.execute();
        profiler.endPhase(LoopProfiler.SCHEDULER_EXECUTE);

        // ── Phase: loopEX ──
        profiler.startPhase();
        loopEX();
        profiler.endPhase(LoopProfiler.LOOP_EX);

        // ── Phase: telemetry ──
        profiler.startPhase();
        loopTime = timer.milliseconds() - lastTime;
        profiler.update(telemetry);
        profiler.endPhase(LoopProfiler.TELEMETRY);

        // ── Record totalLoop ──
        profiler.endTotalLoop();
    }

    /**
     * Use this to write data to the log
     */
    @Override
    public void stop() {

    }
}