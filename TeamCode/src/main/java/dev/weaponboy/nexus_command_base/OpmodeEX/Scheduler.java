package dev.weaponboy.nexus_command_base.OpmodeEX;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;

public class Scheduler {

    OpMode opMode;
    SubSystem[] subSystems;

    public Scheduler(OpMode opMode, SubSystem[] subSystems){
        this.opMode = opMode;
        this.subSystems = subSystems;
    }

    public void init(){
        for (int i = 0; i <= subSystems.length-1; i++){
            subSystems[i].init();
        }
    }

    public void execute(){
        for (int i = 0; i <= subSystems.length-1; i++){
            subSystems[i].execute();
        }
    }

}
