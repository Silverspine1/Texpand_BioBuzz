package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing.PathingUtility;

public class PIDController {

    private double kP;
    private double kI;
    private double kD;

    private double previousError;
    private double integral;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setI(double kI){
        this.kI = kI;
    }

    public double calculate(double setpoint, double current) {
        double error = setpoint - current;

        // Proportional term
        double proportional = kP * error;

        // Integral term
        integral += error;
        double integralTerm = kI * integral;

        // Derivative term
        double derivative = kD * (error - previousError);

        // Update the previous error
        previousError = error;

        // Calculate the output
        double output = proportional + integralTerm + derivative;

        return output;
    }

    public double calculate(double error) {
        // Proportional term
        double proportional = kP * error;

        // Integral term
        integral += error;
        double integralTerm = kI * integral;

        // Derivative term
        double derivative = kD * (error - previousError);

        // Update the previous error
        previousError = error;

        // Calculate the output
        return proportional + integralTerm + derivative;
    }

    public void reset() {
        previousError = 0;
        integral = 0;
    }

}
