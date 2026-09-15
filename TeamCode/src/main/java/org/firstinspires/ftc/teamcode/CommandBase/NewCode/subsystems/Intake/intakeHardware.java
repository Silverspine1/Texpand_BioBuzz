package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Intake;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Drivebase.DriveBase;

import dev.weaponboy.nexus_command_base.Hardware.MotorEx;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;

public class intakeHardware extends SubSystem {

    DcMotorEx IMotor;

    enum intakeState {
        off,
        idle,
        intaking,
        ejecting,
        transfering

    }
    intakeState State = intakeState.off;

    double power = IMotor.getPower();

    //double speed = IMotor.getSpeed()

    @Override
    public void init() {
        IMotor = hardwareMap.get(DcMotorEx.class,"Intake");
        IMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void execute() {

        IMotor.update(power);

        //planed to control intake with gamepad??
        if(State == intakeState.off) {

            IMotor.setPower(0);
        } else if (State == intakeState.idle) {

            // x place holder for intaking speed
            if (speed < x) {
                intakeState State = intakeState.idle;
            } else if (speed >= x) {
                intakeState State = intakeState.intaking;
            }
        } else if (State == intakeState.intaking) {

            IMotor.setPower(1);
            if (speed < x) {
                intakeState State = intakeState.idle;
            } else if (speed >= x) {
                intakeState State = intakeState.intaking;
            }
        } else if (State == intakeState.ejecting){
            IMotor.setPower(-1);
        } else if (State == intakeState.transfering){
            if () {

            }
        } else {
            intakeState State = intakeState.off;
        }

    }




}
