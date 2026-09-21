package org.firstinspires.ftc.teamcode.CommandBase.NewCode.State;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Intake.intakeHardware;

import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;

public class RobotState extends SubSystem {

    intakeHardware IntakeHardware;
    public enum IntakeDirState{
        INTAKING,
        EJECTING

    }

    IntakeDirState intakeDirState;

    public enum IntakeAutoOrManual{
        MANUAL,
        AUTO
    }

    public enum FlywheelState{
        ON_SPEED,
        SPEEDING_UP,
        IDLE

    }

    public enum FollowingPath{
        FOLLOWING,
        NOT_FOLLOWING,

    }
    public enum Braking{
        BRAKING,
        NOT_BRAKING,

    }
    public enum ActiveLocalisation{
        PINPOINT,
        THREE_WHEEL,

    }

    public double TargetTurretDir;
    public double TargetHoodAngle;
    public double TargetFlywheelSpeed;

    @Override
    public void init()  {

    }

    @Override
    public void execute() {

    }

}
