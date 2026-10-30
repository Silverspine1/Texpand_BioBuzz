package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Follower;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Localisation.Odometry;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_command_base.Commands.Execute;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.Follower.Follower;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathGeneration.PathsManager;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathGeneration.commands.SectionBuilder;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathingUtility.RobotPower;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.RobotUtilities.RobotConfig;
import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.RobotUtilities.Vector2D;


@TeleOp(name = "Direction Test", group = "test")
public class DirectionTest extends OpModeEX {

    PathsManager paths = new PathsManager(new RobotConfig());
    Follower follow = new Follower(new RobotConfig());

    private static final double POWER = 0.3;
    private RobotConfig genConfig;
    private Follower safeFollow;
    boolean pathing = false;
    double heading =0;
    private final SectionBuilder[] shoot = new SectionBuilder[] {
            () -> paths.addPoints(new Vector2D(0, 0), new Vector2D(62.5, 62.5)),
    };


    @Override
    public void initEX() {
        paths.addNewPath("shoot");
        paths.buildPath(shoot);

    }

    @Override
    public void loopEX() {


        double vertical = 0;
        double horizontal = 0;
        double pivot = 0;
        String expected = "-";

        if (gamepad1.dpad_up) {
            vertical = POWER;
            expected = "drive FORWARD, X increases";
        } else if (gamepad1.dpad_down) {
            vertical = -POWER;
            expected = "drive BACKWARD, X decreases";
        } else if (gamepad1.dpad_left) {
            horizontal = -POWER;
            expected = "strafe LEFT, Y decreases";
        } else if (gamepad1.dpad_right) {
            horizontal = POWER;
            expected = "strafe RIGHT, Y  increases";
        } else if (gamepad1.right_bumper) {
            pivot = POWER;
            expected = "turn CLOCKWISE, heading decreases";
        } else if (gamepad1.left_bumper) {
            pivot = -POWER;
            expected = "turn ANTICLOCKWISE, heading increases";
        }
        if (!pathing && gamepad1.leftBumperWasPressed()){
            pathing = true;
            follow.setPath(paths.returnPath("shoot"));
        }else if (pathing && gamepad1.leftBumperWasPressed()){
                pathing = false;
            }

if (!pathing) {
    driveBase.drivePowers(vertical, horizontal, pivot);
}


        telemetry.addData("Expected", expected);
        telemetry.addData("X (cm)", odometry.X());
        telemetry.addData("Y (cm)", odometry.Y());
        telemetry.addData("Heading (deg)", odometry.Heading());
        telemetry.addData("Velocity X (cm/s)", odometry.getXVelocity());
        telemetry.addData("Velocity Y (cm/s)", odometry.getYVelocity());
        if (pathing) {
            RobotPower cp = follow.followPathAuto(heading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity(), odometry.getHeadingVelocityDeg());
            driveBase.queueCommand(driveBase.drivePowers(cp));
        }
    }
}
