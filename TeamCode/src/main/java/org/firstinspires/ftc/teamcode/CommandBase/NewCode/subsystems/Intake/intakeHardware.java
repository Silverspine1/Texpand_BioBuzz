package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Intake;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Drivebase.DriveBase;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_command_base.Commands.Command;
import dev.weaponboy.nexus_command_base.Hardware.MotorEx;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;

public class intakeHardware extends SubSystem {

    MotorEx IMotor = new MotorEx();
    public intakeHardware(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, returnDefaultCommand());
    }


    public enum intakeState {
        off,
        intaking,
        ejecting,
        transferring,
        holding

    }

    ElapsedTime ReverseTimer = new ElapsedTime();
    ElapsedTime IntakeTimer = new ElapsedTime();
    public intakeState State = intakeState.off;
    boolean intakeAfterReverse = false;
    boolean reverse;
    double reverseTime;
    double intakeTime;

    double currentPower = 0;

    @Override
    public void init() {

        IMotor.initMotor("IMotor", getOpMode().hardwareMap);
    }

    @Override
    public void execute() {
        //controls the motor relative to the state which will be set by a different loop
        switch (State){
            case off:
                currentPower = 0;

                break;
            case intaking:
                currentPower = 1;

                break;
            case ejecting:
                currentPower = -1;
                break;
            case transferring:
                currentPower = -1;
                break;

        }

        IMotor.update(currentPower);

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

    public void Reverse(boolean Reverse, double ReverseTime, double IntakeTime){
        this.intakeTime = IntakeTime;
        this.reverse = Reverse;
        this.reverseTime = ReverseTime;

    }
}
