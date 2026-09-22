package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Intake;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_command_base.Commands.LambdaCommand;
import dev.weaponboy.nexus_command_base.Hardware.MotorEx;
import dev.weaponboy.nexus_command_base.Hardware.ServoDegrees;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;

public class Transfer extends SubSystem {

    enum transferState {
        open,
        closed
    }

    MotorEx TMotor = new MotorEx();

    double closed = 70;
    double open = 225;
    intakeHardware IntakeHardware;
    transferState Transfer = transferState.closed;
    ServoDegrees blocker = new ServoDegrees();
    public boolean transfer = false;
    double transferTime = 500;
    ElapsedTime TransferTimer = new ElapsedTime();

    public Transfer(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, new LambdaCommand(() -> {}, () -> {}, () -> true));
        IntakeHardware = opModeEX.IntakeHardware;
    }

    @Override
    public void init() {
        blocker.setRange(255);
        TMotor.initMotor("TMotor", getOpMode().hardwareMap);
    }

    @Override
    public void execute() {
        executeEX();

        if (transfer && TransferTimer.milliseconds() > transferTime) {
            IntakeHardware.State = intakeHardware.intakeState.off;
            transfer = false;
            TMotor.update(0);
        }
    }

    public void Trans(boolean transfer, double transferTime) {
        this.transfer = transfer;
        this.transferTime = transferTime;

         if (transfer) {
            TransferTimer.reset();
            TMotor.update(-1);
            IntakeHardware.State = intakeHardware.intakeState.intaking;
        }
    }
}