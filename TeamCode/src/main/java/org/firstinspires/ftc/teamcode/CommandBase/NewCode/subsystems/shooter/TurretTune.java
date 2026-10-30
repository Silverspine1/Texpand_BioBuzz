package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Properties;

/** Turret controller gains, saved to FIRST/settings/turret_tune.properties on the Control Hub. */
public class TurretTune {

    public static final String FILE_NAME = "turret_tune.properties";

    // Files from an older, incompatible tuner are ignored rather than loaded.
    private static final String FORMAT_VERSION = "2";

    public double kS;
    public double kV;
    public double kA;
    public double kP;
    public double kVelocityFeedback;
    public double maxCorrection;
    public double maxVelocity;
    public double maxAcceleration;
    public double latency;

    /** Speed-demand control for the continuous-rotation servos (see SetTurretAngle). Off = trapezoid profile. */
    public boolean brakeAware;
    public double brakeDelayScale = 1.0;

    /** Predictive-mode plant: delay (s), speed slew up and back to neutral (deg/s^2), position gain (1/s), stopping margin. */
    public double plantDelay = 0.04;
    public double plantAccel = 1500;
    public double plantDecel = 1500;
    public double kPos = 15;
    public double decelSafety = 0.8;

    /** Measured speed (deg/s) -> servo power curve. Null means use kS + kV * speed. */
    public double[] ffSpeed;
    public double[] ffPower;

    public TurretTune copy() {
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
        t.brakeAware = brakeAware;
        t.brakeDelayScale = brakeDelayScale;
        t.plantDelay = plantDelay;
        t.plantAccel = plantAccel;
        t.plantDecel = plantDecel;
        t.kPos = kPos;
        t.decelSafety = decelSafety;
        t.ffSpeed = ffSpeed == null ? null : ffSpeed.clone();
        t.ffPower = ffPower == null ? null : ffPower.clone();
        return t;
    }

    public void applyTo(SetTurretAngle turret) {
        turret.setGains(kS, kV, kA, kP, kVelocityFeedback, maxCorrection);
        turret.setConstraints(maxVelocity, maxAcceleration);
        turret.setLatencyCompensation(latency);
        turret.setFeedforwardTable(ffSpeed, ffPower);
        turret.setPlant(plantDelay, plantAccel, plantDecel, kPos, decelSafety);
        turret.setBrakeAware(brakeAware, brakeDelayScale);
    }

    public boolean isValid() {
        double[] values = {kS, kV, kA, kP, kVelocityFeedback, maxCorrection, maxVelocity, maxAcceleration, latency,
                brakeDelayScale, plantDelay, plantAccel, plantDecel, kPos, decelSafety};
        for (double v : values) {
            if (Double.isNaN(v) || Double.isInfinite(v) || v < 0) return false;
        }
        return kV > 0 && maxVelocity > 0 && maxAcceleration > 0
                && maxCorrection > 0 && maxCorrection <= 0.5 && latency <= 0.5
                && isValidTable(ffSpeed, ffPower);
    }

    public static boolean isValidTable(double[] speed, double[] power) {
        if (speed == null && power == null) return true;
        if (speed == null || power == null || speed.length != power.length || speed.length < 2) return false;
        for (int i = 0; i < speed.length; i++) {
            if (!Double.isFinite(speed[i]) || !Double.isFinite(power[i]) || speed[i] < 0 || power[i] < 0) return false;
            if (i > 0 && (speed[i] <= speed[i - 1] || power[i] < power[i - 1])) return false;
        }
        return true;
    }

    public static File defaultFile() {
        return AppUtil.getInstance().getSettingsFile(FILE_NAME);
    }

    public void save() throws IOException {
        save(defaultFile());
    }

    public void save(File file) throws IOException {
        Properties p = new Properties();
        p.setProperty("version", FORMAT_VERSION);
        p.setProperty("kS", Double.toString(kS));
        p.setProperty("kV", Double.toString(kV));
        p.setProperty("kA", Double.toString(kA));
        p.setProperty("kP", Double.toString(kP));
        p.setProperty("kVelocityFeedback", Double.toString(kVelocityFeedback));
        p.setProperty("maxCorrection", Double.toString(maxCorrection));
        p.setProperty("maxVelocity", Double.toString(maxVelocity));
        p.setProperty("maxAcceleration", Double.toString(maxAcceleration));
        p.setProperty("latency", Double.toString(latency));
        p.setProperty("brakeAware", Boolean.toString(brakeAware));
        p.setProperty("brakeDelayScale", Double.toString(brakeDelayScale));
        p.setProperty("plantDelay", Double.toString(plantDelay));
        p.setProperty("plantAccel", Double.toString(plantAccel));
        p.setProperty("plantDecel", Double.toString(plantDecel));
        p.setProperty("kPos", Double.toString(kPos));
        p.setProperty("decelSafety", Double.toString(decelSafety));
        if (ffSpeed != null) {
            StringBuilder table = new StringBuilder();
            for (int i = 0; i < ffSpeed.length; i++) {
                if (i > 0) table.append(',');
                table.append(ffSpeed[i]).append(':').append(ffPower[i]);
            }
            p.setProperty("ffTable", table.toString());
        }

        File parent = file.getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs()) {
            throw new IOException("Could not create " + parent);
        }
        try (OutputStream out = new FileOutputStream(file)) {
            p.store(out, "Turret tune");
        }
    }

    /** Returns null if there is no saved tune or it can't be read. */
    public static TurretTune load() {
        try {
            return load(defaultFile());
        } catch (Throwable t) {
            return null;
        }
    }

    public static TurretTune load(File file) {
        if (file == null || !file.exists()) return null;

        Properties p = new Properties();
        try (InputStream in = new FileInputStream(file)) {
            p.load(in);
            if (!FORMAT_VERSION.equals(p.getProperty("version"))) return null;

            TurretTune t = new TurretTune();
            t.kS = Double.parseDouble(p.getProperty("kS"));
            t.kV = Double.parseDouble(p.getProperty("kV"));
            t.kA = Double.parseDouble(p.getProperty("kA"));
            t.kP = Double.parseDouble(p.getProperty("kP"));
            t.kVelocityFeedback = Double.parseDouble(p.getProperty("kVelocityFeedback"));
            t.maxCorrection = Double.parseDouble(p.getProperty("maxCorrection"));
            t.maxVelocity = Double.parseDouble(p.getProperty("maxVelocity"));
            t.maxAcceleration = Double.parseDouble(p.getProperty("maxAcceleration"));
            t.latency = Double.parseDouble(p.getProperty("latency", "0"));
            t.brakeAware = Boolean.parseBoolean(p.getProperty("brakeAware", "false"));
            t.brakeDelayScale = Double.parseDouble(p.getProperty("brakeDelayScale", "1.0"));
            t.plantDelay = Double.parseDouble(p.getProperty("plantDelay", "0.04"));
            t.plantAccel = Double.parseDouble(p.getProperty("plantAccel", "1500"));
            t.plantDecel = Double.parseDouble(p.getProperty("plantDecel", "1500"));
            t.kPos = Double.parseDouble(p.getProperty("kPos", "15"));
            t.decelSafety = Double.parseDouble(p.getProperty("decelSafety", "0.8"));

            String table = p.getProperty("ffTable");
            if (table != null && !table.isEmpty()) {
                String[] entries = table.split(",");
                t.ffSpeed = new double[entries.length];
                t.ffPower = new double[entries.length];
                for (int i = 0; i < entries.length; i++) {
                    String[] pair = entries[i].split(":");
                    t.ffSpeed[i] = Double.parseDouble(pair[0]);
                    t.ffPower[i] = Double.parseDouble(pair[1]);
                }
            }
            return t.isValid() ? t : null;
        } catch (IOException | RuntimeException e) {
            return null;
        }
    }

    @Override
    public String toString() {
        return String.format(
                "kS=%.4f kV=%.6f kA=%.6f kP=%.5f kVelFb=%.6f maxCorr=%.2f maxVel=%.0f maxAcc=%.0f latency=%.3f brake=%s/%.2f table=%s",
                kS, kV, kA, kP, kVelocityFeedback, maxCorrection, maxVelocity, maxAcceleration, latency,
                brakeAware, brakeDelayScale,
                ffSpeed == null ? "none" : ffSpeed.length + "pts");
    }
}
