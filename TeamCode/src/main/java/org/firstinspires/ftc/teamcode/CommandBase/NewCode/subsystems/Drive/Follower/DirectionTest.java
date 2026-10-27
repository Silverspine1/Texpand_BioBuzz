package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Follower;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Localisation.Odometry;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_command_base.Commands.Execute;
import dev.weaponboy.nexus_pathing.Follower.Follower;
import dev.weaponboy.nexus_pathing.PathGeneration.PathsManager;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.SectionBuilder;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.RobotConfig;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;


@TeleOp(name = "Direction Test", group = "test")
public class DirectionTest extends OpModeEX {

    PathsManager paths = new PathsManager(new RobotConfig()
            .setXLastAdjustmentPD(0.02, 0.004)
            .setYLastAdjustmentPD(0.02, 0.009)
            .setXOnPathPD(0.08, 0.004)
            .setYOnPathPD(0.2, 0.004)
            .setFastHeadingPD(0.01, 0.0005)
            .setSlowHeadingPD(0.012, 0.002)
            .setRobotConstants(130, 181, 650, 700));
    Follower follow = new Follower(new RobotConfig()
            .setXLastAdjustmentPD(0.02, 0.004)
            .setYLastAdjustmentPD(0.02, 0.009)
            .setXOnPathPD(0.08, 0.004)
            .setYOnPathPD(0.2, 0.004)
            .setFastHeadingPD(0.01, 0.0005)
            .setSlowHeadingPD(0.012, 0.002)
            .setRobotConstants(130, 181, 650, 700));

    private static final double POWER = 0.3;
    private RobotConfig genConfig;
    private Follower safeFollow;
    boolean pathing = false;
    double heading =270;
    private final SectionBuilder[] shoot = new SectionBuilder[] {
            () -> paths.addPoints(new Vector2D(0, 0), new Vector2D(40, 20)),
    };


    @Override
    public void initEX() {
        paths.addNewPath("shoot");
        paths.buildPath(shoot);

    }

    @Override
    public void loopEX() {
        follow.setPath(paths.returnPath("shoot"));


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
            RobotPower cp = follow.followPathAuto(heading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
            driveBase.queueCommand(driveBase.drivePowers(cp));
        }
    }
}
