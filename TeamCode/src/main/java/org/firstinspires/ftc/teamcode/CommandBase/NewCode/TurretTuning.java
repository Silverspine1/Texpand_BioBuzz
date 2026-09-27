package org.firstinspires.ftc.teamcode.CommandBase.NewCode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter.TurretTune;

/** Live-editable from FTC Dashboard (192.168.43.1:8080/dash) while Test_tele runs. */
@Config
public class TurretTuning {
    public static double kS = 0.0;
    public static double kV = 0.0020;
    public static double kA = 0.0;
    public static double kP = 0.004;
    public static double kVelocityFeedback = 0.0;
    public static double maxCorrection = 0.4;

    public static double maxVelocity = 220.0;
    public static double maxAcceleration = 1200.0;
    public static double latency = 0.0;

    public static double testAngle = 45.0;
    public static double pingPongSeconds = 1.5;
    public static double manualMaxPower = 0.3;

    /** Auto-tune keeps the turret within +/- this many degrees of where it was at INIT. */
    public static double safeRangeDeg = 110.0;

    public static void copyFrom(TurretTune t) {
        kS = t.kS;
        kV = t.kV;
        kA = t.kA;
        kP = t.kP;
        kVelocityFeedback = t.kVelocityFeedback;
        maxCorrection = t.maxCorrection;
        maxVelocity = t.maxVelocity;
        maxAcceleration = t.maxAcceleration;
        latency = t.latency;
    }

    public static TurretTune toTune() {
        TurretTune t = new TurretTune();
        t.kS = kS;
        t.kV = kV;
        t.kA = kA;
        t.kP = kP;
        t.kVelocityFeedback = kVelocityFeedback;
        t.maxCorrection = maxCorrection;
        t.maxVelocity = maxVelocity;
        t.maxAcceleration = maxAcceleration;
        t.latency = latency;
        return t;
    }
}
