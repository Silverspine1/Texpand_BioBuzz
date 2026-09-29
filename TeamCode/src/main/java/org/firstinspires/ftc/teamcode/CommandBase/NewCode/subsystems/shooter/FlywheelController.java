package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

/** Feedforward + P + trimmed integral flywheel speed control. Returns motor power 0..maxPower. */
public class FlywheelController {

    public double kF = 0.00018;
    public double kP = 0.0003;
    public double kI = 0.0008;
    public double integralMax = 0.25;
    public double maxPower = 1.0;
    public double integralZoneRpm = 300;

    private double integral = 0;

    public void reset() {
        integral = 0;
    }

    public double update(double targetRPM, double measuredRPM, double dt) {
        if (!(targetRPM > 0) || !Double.isFinite(measuredRPM) || !(dt > 0)) {
            integral = 0;
            return 0;
        }

        double error = targetRPM - measuredRPM;
        double step = Math.abs(error) <= integralZoneRpm ? kI * error * dt : 0;
        integral = clamp(integral + step, -integralMax, integralMax);

        double output = kF * targetRPM + kP * error + integral;
        if (output > maxPower) {
            if (error > 0) integral -= step;
            output = maxPower;
        } else if (output < 0) {
            if (error < 0) integral -= step;
            output = 0;
        }
        return output;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
