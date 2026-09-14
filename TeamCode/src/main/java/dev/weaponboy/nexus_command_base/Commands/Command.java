package dev.weaponboy.nexus_command_base.Commands;

public interface Command {

    void execute();

    void init();

    boolean isFinished();

}
