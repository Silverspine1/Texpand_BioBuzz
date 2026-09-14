package org.firstinspires.ftc.teamcode.CommandBase.Subsytems;

import org.firstinspires.ftc.teamcode.CommandBase.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.CommandBase.OpModeEX;
import org.firstinspires.ftc.teamcode.CommandBase.LoopProfiler;
import org.firstinspires.ftc.teamcode.CommandBase.GoBildaPinpointDriver.Register;

import java.util.ArrayList;

import dev.weaponboy.nexus_command_base.Commands.LambdaCommand;
import dev.weaponboy.nexus_command_base.Subsystem.SubSystem;

public class Odometry extends SubSystem {

    public GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;

    Register[] onlyPosition = {
            Register.X_POSITION,
            Register.Y_POSITION,
            Register.H_ORIENTATION,
            Register.X_VELOCITY,
            Register.Y_VELOCITY,
    };

    DcMotorEx leftPod;
    DcMotorEx rightPod;
    DcMotorEx backPod;

    public double X, Y, Heading, normilised;
    double startX, startY, startHeading;

    double XVelocity = 0;
    double YVelocity = 0;
    double HVelocity = 0;
    boolean resetAtStart = false;

    public Odometry(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, update);
    }

    public void startPosition(double X, double Y, int Heading) {
        this.startX = X;
        this.startY = Y;
        this.Heading = Heading;
    }

    @Override
    public void init() {
        odo = getOpMode().hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // --- Pinpoint I2C Optimization (Phase 2) ---
        // Only read what we actually use. Excluding LOOP_TIME, etc.
        odo.setBulkReadScope(
                Register.X_POSITION,
                Register.Y_POSITION,
                Register.H_ORIENTATION,
                Register.X_VELOCITY,
                Register.Y_VELOCITY,
                Register.H_VELOCITY);

        odo.setOffsets(-13, 132, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
    }

    public double headingError(double targetHeading) {
        return Heading - targetHeading;
    }

    @Override
    public void execute() {
        long start = System.nanoTime();
        executeEX();

        ((OpModeEX) getOpMode()).profiler.recordDuration(LoopProfiler.ODOMETRY, System.nanoTime() - start);
    }

    public double X() {
        return X;
    }

    public double Y() {
        return Y;
    }

    public double Heading() {
        return 360 - Heading;
    }

    public double normiliased() {
        return normilised;
    }

    public double getYVelocity() {
        return YVelocity;
    }

    public double getXVelocity() {
        return XVelocity;
    }

    public double getHVelocity() {
        return HVelocity;
    }

    public LambdaCommand update = new LambdaCommand(
            () -> {
            },
            () -> {

                odo.update();

                XVelocity = -odo.getVelX(DistanceUnit.CM);
                YVelocity = odo.getVelY(DistanceUnit.CM);
                HVelocity = odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

                // Cache heading reads (avoids redundant unit conversions)
                double rawDeg = odo.getHeading(AngleUnit.DEGREES);
                double rawRad = odo.getHeading(AngleUnit.RADIANS);

                Heading = startHeading + rawDeg;
                normilised = startHeading + rawRad;

                if (startHeading + rawDeg < 0) {
                    Heading = startHeading + rawDeg + 360;
                } else {
                    Heading = startHeading + rawDeg;
                }

                X = startX - odo.getPosX(DistanceUnit.CM);
                Y = startY + odo.getPosY(DistanceUnit.CM);
            },
            () -> false);

    public void offsetY(double offset) {
        Y += offset;
    }
}