package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Intake;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Drivebase.DriveBase;

import dev.weaponboy.nexus_command_base.Hardware.MotorEx;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;

public class intakeHardware extends SubSystem {

    MotorEx IMotor = new MotorEx();

    enum intakeState {
        off,
        intaking,
        ejecting,
        transfering

    }
    intakeState State = intakeState.off;

    double currentPower = IMotor.getPower();

    //double speed = IMotor.getSpeed()

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

        } else if (State == intakeState.ejecting){

            currentPower = -1;

        } else if (State == intakeState.transfering){
            //depends souly on the mec

        } else {
            intakeState State = intakeState.off;
        }

        IMotor.update(currentPower);
        
    }



}
