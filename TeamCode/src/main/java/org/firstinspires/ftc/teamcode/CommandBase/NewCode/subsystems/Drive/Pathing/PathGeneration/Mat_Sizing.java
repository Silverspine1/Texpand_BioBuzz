package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathGeneration;

public class Mat_Sizing {

    public static final double matSizeAtComp = 60;

    public static final double turnedMatSize = 60;

    /**convert to field size*/
    public static double getRealCoords(double numberToConvert){

        numberToConvert = (numberToConvert/turnedMatSize) * matSizeAtComp;

        return numberToConvert;

    }

}
