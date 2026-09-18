package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Intake;

import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.nexus_command_base.Hardware.ServoDegrees;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;

public class Transfer extends SubSystem {

    enum transferState {
        open,
        closed
    }

    double closed = 70;
    double open = 225;
    intakeHardware IntakeHardware;
    transferState Transfer = transferState.closed;
    ServoDegrees blocker = new ServoDegrees();
    boolean transfer = false;
    double transferTime = 500;
    ElapsedTime TransferTimer = new ElapsedTime();

    @Override
    public void init() {
        blocker.initServo("blocker", getOpMode().hardwareMap);
        blocker.setRange(255);
    }

    @Override
    public void execute() {

        if (transfer) {
            IntakeHardware.State = intakeHardware.intakeState.transfering;
            Transfer = transferState.open;

        } else if (!transfer) {
            Transfer = transferState.closed;
        }
        switch (Transfer) {
            case open:
                blocker.setPosition(open);

                break;
            case closed:
                blocker.setPosition(closed);

                break;


        }
        if (TransferTimer.milliseconds() > transferTime && transfer) {

            TransferTimer.reset();
            IntakeHardware.State = intakeHardware.intakeState.transfering;

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

