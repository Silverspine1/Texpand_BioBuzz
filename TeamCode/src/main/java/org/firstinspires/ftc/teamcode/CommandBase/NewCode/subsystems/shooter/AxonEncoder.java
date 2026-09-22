package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AxonEncoder {

    AnalogInput encoder;
    double difference = 0;
    double previousAngle = 0;
    double totalPosition = 0;
    boolean updateTurretPosition = true;
    double gearRatio = 2.34;
    double degreesPerSecond = 0;
    ElapsedTime veloTimer = new ElapsedTime();



    public void setOffset(double offset) {
        this.offset = offset;
    }

    double offset = 0;

    public void init(HardwareMap hardwareMap, String deviceName){
        encoder = hardwareMap.get(AnalogInput.class, deviceName);
        previousAngle = encoder.getVoltage() / 3.3 * 360;
    }


    double getPosition(){
        return (encoder.getVoltage() / 3.3 * 360) + offset;
    }
    public void UpdatePosition() {
        difference = getPosition() - previousAngle;

        if (difference > 180) {
            difference -= 360;
        } else if (difference < -180) {
            difference += 360;
        }
        degreesPerSecond = difference/(veloTimer.nanoseconds()/1000000000);
        veloTimer.reset();
        previousAngle = getPosition();
        totalPosition += difference;

    }
    public void resetTurretAngle(){
        totalPosition = 0;
    }
    public double getTurretAngle(){
        return -totalPosition / gearRatio;
    }
    public double getVelocity() {
        return -((degreesPerSecond+0.000001) / gearRatio);
    }



}