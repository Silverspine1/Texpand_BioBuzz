package dev.weaponboy.nexus_command_base.Subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;

import dev.weaponboy.nexus_command_base.Commands.Command;


@SuppressWarnings("unused")
public abstract class SubSystem implements SubSystemInt{

    OpMode opMode;

    Command defaultCommand;
    Command currentCommand;
    ArrayList<Command> commands = new ArrayList<>();

    public void registerSubsystem(OpMode opMode, Command defaultCommand){
        this.opMode = opMode;
        this.defaultCommand = defaultCommand;
        this.currentCommand = defaultCommand;
    }

    protected OpMode getOpMode() {
        return opMode;
    }

    protected void executeEX(){

        if (currentCommand.isFinished() && !commands.isEmpty()){
            currentCommand = commands.get(0);
            commands.remove(currentCommand);
            currentCommand.init();
        }else if (currentCommand.isFinished() && commands.isEmpty()){
            currentCommand = defaultCommand;
            currentCommand.init();
        }

        currentCommand.execute();
    }

    @Override
    public void setDefaultCommand(Command defaultCommand) {
        this.defaultCommand = defaultCommand;
    }

    @Override
    public Command returnDefaultCommand() {
        return currentCommand;
    }

    @Override
    public Command getCurrentCommand() {
        return currentCommand;
    }

    @Override
    public void queueCommand(Command command){
        commands.add(command);
    }

    @Override
    public void clearQueue(){
        commands.clear();
    }

    @Override
    public void overrideCurrent(boolean override, Command command){
        if (override){
            commands.clear();
            currentCommand = command;
        }
    }

}
