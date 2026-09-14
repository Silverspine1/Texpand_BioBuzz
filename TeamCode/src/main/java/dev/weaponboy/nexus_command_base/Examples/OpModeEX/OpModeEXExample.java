package dev.weaponboy.nexus_command_base.Examples.OpModeEX;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.nexus_command_base.Examples.SubSystems.Subsystem;
import dev.weaponboy.nexus_command_base.OpmodeEX.Scheduler;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;

import java.util.List;

public abstract class OpModeEXExample extends OpMode {

    public Subsystem exampleSubsystem = new Subsystem(this);

    private final Scheduler scheduler = new Scheduler(this, new SubSystem[] {exampleSubsystem});

    List<LynxModule> allHubs;

    public ElapsedTime autoTime = new ElapsedTime();

    ElapsedTime timer = new ElapsedTime();
    double lastTime;
    public double loopTime;

    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad lastGamepad1 = new Gamepad();

    public Gamepad currentGamepad2 = new Gamepad();
    public Gamepad lastGamepad2 = new Gamepad();

    public abstract void initEX();

    public abstract void loopEX();

    @Override
    public void init() {

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

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

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        lastGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        lastGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);

        lastTime = timer.milliseconds();

        scheduler.execute();
        loopEX();

        loopTime = timer.milliseconds() - lastTime;
        System.out.println("loop time: " + loopTime);
    }

    /**
     * Use this to write data to the log
     * */
    @Override
    public void stop() {

    }
}
