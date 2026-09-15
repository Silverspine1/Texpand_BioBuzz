package org.firstinspires.ftc.teamcode.CommandBase.NewCode.State;

public class FealdState {


    public enum LonchZoneState{
        IN_ZONE,
        OUT_ZONE

    }


    public enum HiveSide{
        lEFT,
        RIGHT

    }


    public enum RedyTosoot{
        REDY,
        NOT_REDY,

    }



    // Game elamints count
    public int NumGameElemnts;
    public int NumNecter;
    public int NumPolen;


    // Robot feald pos
    public double FealdX;
    public double FealdY;
    public double RobotDir;



    //Velosity For robot In feald sentric
    public double FealdXVel;
    public double FealdYVel;
    public double RobotDirVel;


    //Target Pos
    public double TargetX;
    public double TargetY;
    public double TargetDir;

    //Posable pikup
    public double NumberOfPosablePikups;
    //Todo add a List



}
