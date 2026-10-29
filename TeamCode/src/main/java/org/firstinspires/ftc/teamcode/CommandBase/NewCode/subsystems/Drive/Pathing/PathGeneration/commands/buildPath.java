package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathGeneration.commands;

import java.util.ArrayList;

public class buildPath{

    public buildPath(SectionBuilder[] commands){

        for (int i = 0; i < commands.length; i++){
            commands[i].buildSection();
        }

    }

}
