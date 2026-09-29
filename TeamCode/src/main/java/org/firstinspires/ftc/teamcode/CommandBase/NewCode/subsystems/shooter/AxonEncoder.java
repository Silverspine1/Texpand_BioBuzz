package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AxonEncoder {

    // Flip this one sign to reverse the turret angle AND velocity together (+ = counter-clockwise from above).
    public static final double TURRET_DIRECTION = -1;

    AnalogInput encoder;
    double maxVoltage = 3.3;
    double difference = 0;
    double previousAngle = 0;
    double totalPosition = 0;
    double gearRatio = 2.34;
    double degreesPerSecond = 0;
    double velocityFilterGain = 0.3;
    ElapsedTime veloTimer = new ElapsedTime();

    double offset = 0;

    public void setOffset(double offset) {
        previousAngle += offset - this.offset;
        this.offset = offset;
    }

    public void init(HardwareMap hardwareMap, String deviceName){
        encoder = hardwareMap.get(AnalogInput.class, deviceName);
        if (encoder.getMaxVoltage() > 0) {
            maxVoltage = encoder.getMaxVoltage();
        }
        previousAngle = getPosition();
        veloTimer.reset();
    }

    double getPosition(){
        return (encoder.getVoltage() / maxVoltage * 360) + offset;
    }

    /** Call exactly once per loop. */
    public void UpdatePosition() {
        double position = getPosition();
        difference = position - previousAngle;

        if (difference > 180) {
            difference -= 360;
        } else if (difference < -180) {
            difference += 360;
        }

        double dt = veloTimer.nanoseconds() / 1e9;
        veloTimer.reset();
        if (dt > 0) {
            double rawVelocity = difference / dt;
            degreesPerSecond += velocityFilterGain * (rawVelocity - degreesPerSecond);
        }

        previousAngle = position;
        totalPosition += difference;
    }
    public void resetTurretAngle(){
        totalPosition = 0;
    }
    private double toTurretDegrees(double servoDegrees) {
        return TURRET_DIRECTION * servoDegrees / gearRatio;
    }

    /** Turret angle in degrees. */
    public double getTurretAngle(){
        return toTurretDegrees(totalPosition);
    }

    /** Turret velocity in degrees per second (the turret itself, not the servo). */
    public double getVelocity() {
        return toTurretDegrees(degreesPerSecond);
    }

    public double getVoltage() {
        return encoder.getVoltage();
    }
    public double getRawDegrees() {
        return encoder.getVoltage() / maxVoltage * 360;
    }
    public double getTotalPosition() {
        return totalPosition;
    }
    public double getLastDifference() {
        return difference;
    }
}
