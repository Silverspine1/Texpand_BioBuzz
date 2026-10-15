package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Follower;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Localisation.Odometry;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;

import dev.weaponboy.nexus_command_base.Commands.Execute;


@TeleOp(name = "Direction Test", group = "test")
public class DirectionTest extends OpModeEX {

    private static final double POWER = 0.3;


    @Override
    public void initEX() {

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

        driveBase.drivePowers(vertical, horizontal, pivot);


        telemetry.addData("Expected", expected);
        telemetry.addData("X (cm)", odometry.X());
        telemetry.addData("Y (cm)", odometry.Y());
        telemetry.addData("Heading (deg)", odometry.Heading());
        telemetry.addData("Velocity X (cm/s)", odometry.getXVelocity());
        telemetry.addData("Velocity Y (cm/s)", odometry.getYVelocity());
    }
}
