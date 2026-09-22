package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Intake;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

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
        registerSubsystem(opModeEX, returnDefaultCommand());
        IntakeHardware = opModeEX.IntakeHardware;
    }

    @Override
    public void init() {
        blocker.setRange(255);
        TMotor.initMotor("TMotor", getOpMode().hardwareMap);

    }

    @Override
    public void execute() {

        if (transfer) {
            IntakeHardware.State = intakeHardware.intakeState.intaking;
            TMotor.update(1);

        } else if (!transfer) {
            TMotor.update(0);
        }
        switch (Transfer) {
            case open:

                break;
            case closed:

                break;


        }
        if (TransferTimer.milliseconds() > transferTime && transfer) {

            TransferTimer.reset();
            IntakeHardware.State = intakeHardware.intakeState.intaking;

        } else if (transfer && TransferTimer.milliseconds() > transferTime) {

            IntakeHardware.State = intakeHardware.intakeState.off;
            transfer = false;
        }

    }
    public void Transfer(boolean transfer, double transferTime){
        this.transfer = transfer;
        this.transferTime = transferTime;

    }
}

