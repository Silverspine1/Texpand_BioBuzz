package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Intake;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Drivebase.DriveBase;

import dev.weaponboy.nexus_command_base.Commands.Command;
import dev.weaponboy.nexus_command_base.Hardware.MotorEx;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;

public class intakeHardware extends SubSystem {

    MotorEx IMotor = new MotorEx();

    enum intakeState {
        off,
        intaking,
        ejecting,
        transfering

    }

    ElapsedTime ReverseTimer = new ElapsedTime();
    ElapsedTime IntakeTimer = new ElapsedTime();
    intakeState State = intakeState.off;
    boolean intakeAfterReverse =false;

    double currentPower = IMotor.getPower();

    @Override
    public void init() {
        IMotor.initMotor("IMotor", getOpMode().hardwareMap);
    }

    @Override
    public void execute() {

        //controls the motor relative to the state which will be set by a different loop
        if(State == intakeState.off) {

            currentPower = 0;

        } else if (State == intakeState.intaking) {

            currentPower = 1;
            //add vision imput to stop once ball has been intaked and begins to transfer

        } else if (State == intakeState.ejecting){

            currentPower = -1;
            //add vision imput to stop once ball has been ejected and switches to off

        } else if (State == intakeState.transfering){
            //depends souly on the mec

        } else {
            intakeState State = intakeState.off;
        }

        IMotor.update(currentPower);

    }

    public void Reverse(boolean reverse, double reverseTime, double intakeTime){
        while (intakeAfterReverse && (State == intakeState.ejecting || State == intakeState.intaking) && reverse) {
            if (ReverseTimer.milliseconds() > reverseTime && reverse) {
                ReverseTimer.reset();
                State = intakeState.ejecting;
                reverse = false;
            } else if (!reverse && ReverseTimer.milliseconds() > reverseTime && State == intakeState.ejecting) {
                State = intakeState.intaking;
                IntakeTimer.reset();
                intakeAfterReverse = true;
            } else if (intakeAfterReverse && IntakeTimer.milliseconds() > intakeTime) {
                State = intakeState.off;
                intakeAfterReverse = false;
            }

    }

    }
}
