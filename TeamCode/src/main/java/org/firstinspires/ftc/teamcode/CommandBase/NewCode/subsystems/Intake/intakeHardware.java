package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Intake;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Drivebase.DriveBase;

import dev.weaponboy.nexus_command_base.Hardware.MotorEx;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;

public class intakeHardware extends SubSystem {
    MotorEx IMotor = new MotorEx();

    enum intakeState {
        off,
        idle,
        intaking,
        ejecting,
        transfering

    }
    intakeState State = intakeState.off;

    double power = IMotor.update() ;


    @Override
    public void init() {

    }

    @Override
    public void execute() {

        IMotor.update(power);

        if(State == intakeState.off) {
            //off
        } else if (State == intakeState.idle) {
            //on (not up to speed??)
        } else if (State == intakeState.intaking) {
            //on more power?
        } else if (State == intakeState.ejecting){
            // im not sure
        } else if (State == intakeState.transfering){
            //still not sure...
        }

    }




}

