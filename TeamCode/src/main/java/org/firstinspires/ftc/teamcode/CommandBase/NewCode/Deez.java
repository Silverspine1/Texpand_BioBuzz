
package org.firstinspires.ftc.teamcode.CommandBase.NewCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp
public class Deez extends OpMode {

    // ---------- Shooter controller tuning (live on FTC Dashboard) ----------
    public static double kS = 0.0;       // static friction power (optional)
    public static double kV = 0.00015;    // power per tick/sec  <-- tune first
    public static double kP = 0.0035;    // recovery speed      <-- tune second
    public static double kI = 0.0;       // only if it settles below target
    public static double kD = 0.0;       // usually leave at 0
    public static double I_ZONE = 150;   // ticks/sec, integral only active inside this
    public static double I_MAX = 0.15;   // clamp on integral contribution
    public static double FILTER = 0.3;   // 0 = no filter, higher = smoother but laggier
    public static double POWER_GRAPH_SCALE = 5000; // 1.0 power plots as this many RPM on the graph

    // ---------- Hardware ----------
    private FtcDashboard dashboard;
    private VoltageSensor battery;
    DigitalChannel one;
    private static final double MOTOR_TICKS_PER_REV = 32;

    DcMotorEx TestMotter, TestMotter1, TestMotter2,TestMotter3;
    Servo flower;
    DcMotor frontLeft, frontRight, backLeft, backRight;

    // ---------- State ----------
    double MaxSpeed = 0;
    int TargetSpeed = 100;   // ticks/sec, positive = shooting
    int TargetSpeed2 = 100;   // ticks/sec, positive = shooting

    double Velo;
    double pow = 0.5;
    boolean intaking = false;

    double DEADBAND = 0.1;
    double MAX_POWER_CHANGE_PER_LOOP = 0.07;
    double flPower = 0, frPower = 0, blPower = 0, brPower = 0;

    // Shooter controller state
    private final ElapsedTime ctrlTimer = new ElapsedTime();
    private final ElapsedTime voltTimer = new ElapsedTime();
    private double lastTime = 0, lastError = 0, integral = 0, filtered = 0;
    private double voltage = 12.0;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); // faster loop = less lag
        }
        battery = hardwareMap.voltageSensor.iterator().next();
        voltage = battery.getVoltage();
        one = hardwareMap.get(DigitalChannel.class,"one");
        one.setMode(DigitalChannel.Mode.INPUT);

        TestMotter = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        TestMotter3 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");

        TestMotter1 = hardwareMap.get(DcMotorEx.class, "IMotor");
        TestMotter2 = hardwareMap.get(DcMotorEx.class, "TMotor");
        flower = hardwareMap.get(Servo.class, "flowerServo");

        // Bypass the built-in velocity PID, we do our own
        TestMotter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TestMotter.setDirection(DcMotorSimple.Direction.FORWARD);
        TestMotter2.setDirection(DcMotorSimple.Direction.REVERSE);

        TestMotter3.setDirection(DcMotorSimple.Direction.FORWARD); // positive target = shooting direction
// positive target = shooting direction



        frontLeft  = hardwareMap.dcMotor.get("LF");
        frontRight = hardwareMap.dcMotor.get("RF");
        backLeft   = hardwareMap.dcMotor.get("LB");
        backRight  = hardwareMap.dcMotor.get("RB");
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        ctrlTimer.reset();
        voltTimer.reset();
    }

    double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) return 0;
        return value;
    }

    double rateLimit(double current, double target, double maxChange) {
        if (target == 0) return 0;
        double delta = target - current;
        if (Math.abs(delta) <= maxChange) return target;
        return current + Math.copySign(maxChange, delta);
    }

    /** Feedforward + P (+ small I near target) + voltage comp. Returns motor power. */
    double shooterUpdate(double target, double measured) {
        double now = ctrlTimer.seconds();
        double dt = Math.max(now - lastTime, 1e-3);
        lastTime = now;

        if (voltTimer.seconds() > 0.5) {   // voltage reads are slow, only every 0.5s
            voltage = battery.getVoltage();
            voltTimer.reset();
        }

        if (target == 0) {                 // shooter off
            integral = 0;
            lastError = 0;
            filtered = measured;
            return 0;
        }

        filtered = FILTER * filtered + (1 - FILTER) * measured;
        double error = target - filtered;

        if (Math.abs(error) < I_ZONE) {
            integral += error * dt * kI;
            integral = Math.max(-I_MAX, Math.min(I_MAX, integral));
        } else {
            integral = 0;                  // no windup during big dips
        }

        double derivative = (error - lastError) / dt;
        lastError = error;

        double ff = kS * Math.signum(target) + kV * target;
        double out = ff + kP * error + integral + kD * derivative;

        out *= 12.0 / voltage;             // voltage compensation
        return Math.max(-1, Math.min(1, out));
    }

    @Override
    public void loop() {


        // Target speed adjust
        if (gamepad1.dpadRightWasPressed()) {
            TargetSpeed += 40;
        } else if (gamepad1.dpadLeftWasPressed()) {
            TargetSpeed -= 40;
        }

        if (gamepad1.dpadUpWasPressed()) {
            TargetSpeed2 += 40;
        } else if (gamepad1.dpadDownWasPressed()) {
            TargetSpeed2 -= 40;
        }
        TargetSpeed = Math.max(0, TargetSpeed);

        // Shooter
        double shooterVelo = TestMotter.getVelocity();
        double shooterVelo2 = -TestMotter2.getVelocity();

        double shooterPower = shooterUpdate(TargetSpeed, shooterVelo);
        TestMotter.setPower(shooterUpdate(TargetSpeed,shooterVelo));
        TestMotter3.setPower(shooterUpdate(-TargetSpeed2,shooterVelo2));


        // Intake / transfer
        if (gamepad1.right_bumper) {
            TestMotter1.setPower(1);
        } else {
            TestMotter1.setPower(0);
        }
        if (gamepad1.left_bumper) {
            TestMotter2.setPower(1);
            intaking = true;

        } else  {
            TestMotter2.setPower(0);
            intaking = false;

        }

        // Flower servo
        if (gamepad1.a) {
            pow = 0.5;
        }
        if (gamepad1.b) {
            pow = 0.72;
        }
        flower.setPosition(pow);

        // Drive
        double y = applyDeadband(gamepad1.right_stick_y, DEADBAND);
        double x = -applyDeadband(gamepad1.right_stick_x, DEADBAND);
        double rt = applyDeadband(gamepad1.right_trigger, DEADBAND);
        double lt = applyDeadband(gamepad1.left_trigger, DEADBAND);
        double turn = -rt + lt;

        double flTarget = y + x + turn;
        double blTarget = y - x + turn;
        double frTarget = y - x - turn;
        double brTarget = y + x - turn;

        flPower = rateLimit(flPower, flTarget, MAX_POWER_CHANGE_PER_LOOP);
        blPower = rateLimit(blPower, blTarget, MAX_POWER_CHANGE_PER_LOOP);
        frPower = rateLimit(frPower, frTarget, MAX_POWER_CHANGE_PER_LOOP);
        brPower = rateLimit(brPower, brTarget, MAX_POWER_CHANGE_PER_LOOP);

        frontLeft.setPower(flPower);
        backLeft.setPower(blPower);
        frontRight.setPower(frPower);
        backRight.setPower(brPower);

        if (Math.abs(Velo) > Math.abs(MaxSpeed)) {
            MaxSpeed = Velo;
        }

        // Dashboard graph
        double shooterRpm = (shooterVelo / MOTOR_TICKS_PER_REV) * 60.0;
        double targetRpm = (TargetSpeed / MOTOR_TICKS_PER_REV) * 60.0;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target RPM", targetRpm);
        packet.put("Shooter Actual RPM", shooterRpm);
        packet.put("Power (scaled to RPM)", shooterPower * POWER_GRAPH_SCALE);
        packet.put("Shooter power %", shooterPower * 100);
        dashboard.sendTelemetryPacket(packet);

        // Driver Hub telemetry
        telemetry.addData("one", one.getState());

        telemetry.addData("Target speed", TargetSpeed);
        telemetry.addData("Shooter speed", shooterVelo);
        telemetry.addData("Shooter speed2", shooterVelo2);

        telemetry.addData("Shooter RPM", shooterRpm);
        telemetry.addData("Shooter power", shooterPower);
        telemetry.addData("Battery V", voltage);
        telemetry.addData("Drive speed", Velo);
        telemetry.addData("Max Drive speed", MaxSpeed);
        telemetry.update();
    }
}