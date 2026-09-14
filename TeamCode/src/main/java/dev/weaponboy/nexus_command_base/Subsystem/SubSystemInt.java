package dev.weaponboy.nexus_command_base.Subsystem;


import dev.weaponboy.nexus_command_base.Commands.Command;

public interface SubSystemInt {

    void init();

    void execute();

    Command returnDefaultCommand();

    void setDefaultCommand(Command defaultCommand);

    Command getCurrentCommand();

    void queueCommand(Command command);

    void clearQueue();

    void overrideCurrent(boolean override, Command command);

}
