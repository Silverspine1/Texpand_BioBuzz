package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Follower;

/**
 * Breakaway powers and stop tolerances PathTuner measures. Translation and
 * heading are both driven from the measured drive model by the Follower, so
 * there are no PD gains to compute any more.
 */
public class ModelTuner {

    public double ksFwd = 0, ksStr = 0, ksTurn = 0;
    public double tolX = 3.0, tolY = 3.0, tolH = 3.0;   // cm, cm, deg
}
