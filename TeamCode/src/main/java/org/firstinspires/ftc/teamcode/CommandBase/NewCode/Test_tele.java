package org.firstinspires.ftc.teamcode.CommandBase.NewCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Intake.intakeHardware;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;
@TeleOp
public class Test_tele extends OpModeEX {
    @Override
    public void initEX() {

    }

    @Override
    public void loopEX() {
        driveBase.drivePowers(gamepad1.right_stick_y, (gamepad1.left_trigger- gamepad1.right_trigger),gamepad1.left_stick_x);
        if (gamepad1.right_bumper){
            IntakeHardware.State = intakeHardware.intakeState.intaking;
        }else if(!transfer.transfer) {
            IntakeHardware.State = intakeHardware.intakeState.off;
        }
        if (!lastGamepad1.left_bumper && currentGamepad1.left_bumper){
            transfer.Transfer(true,500);
        }
        if (!lastGamepad1.dpad_up && currentGamepad1.dpad_up && !shooterController.targeting){
            shooterController.targeting = true;
        } else if (!lastGamepad1.dpad_up && currentGamepad1.dpad_up && shooterController.targeting) {
            shooterController.targeting = false;
        }
        telemetry.addData("Y", odometry.Y());
        telemetry.addData("X", odometry.X());
        telemetry.addData("H", odometry.Heading());
        telemetry.addData("YVel", odometry.getYVelocity());
        telemetry.addData("XVel", odometry.getXVelocity());
        telemetry.addData("Hvel", odometry.getHeadingVelocity());








    }
}
