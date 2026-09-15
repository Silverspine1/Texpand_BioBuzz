package org.firstinspires.ftc.teamcode.CommandBase.NewCode.State;

public class RobotState {
    public enum IntaceDirState{
        ITAKING,
        EGECTING

    }
    public enum IntakeAutoOrManuale{
        MANUALE,
        AUTO
    }

    public enum FliweelState{
        ON_SPEED,
        SPEEDING_UP,
        IDLE

    }

    public enum FolowingPath{
        FOLOWING,
        NOT_FOLWOING,

    }
    public enum Braking{
        BRAKING,
        NOT_BRAKING,

    }
    public enum ActiveLocolisashon{
        PINPOINT,
        THREE_WEEL,

    }

    public double TargetTuretDir;
    public double TargetHoodAagnle;
    public double TargetFliweelSpeed;




}
