package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

import java.util.Random;

/**
 * Turret + continuous-rotation servos as measured by the tuner's sweep: command -> target speed through the measured
 * table, a transport delay, a slew limit on speed (acceleration, and a separate one back to neutral), stiction on
 * start, and the encoder's noise and velocity filter. Used to score controller gains offline, in simulated time.
 */
public class TurretPlantModel implements SetTurretAngle.Sensor, SetTurretAngle.Sink {

    /** Everything the model needs; filled by the tuner from the sweep. */
    public static class Params {
        public double[] tableSpeed;     // deg/s, ascending, tableSpeed[0] = 0
        public double[] tablePower;     // command offset from neutral that gives that speed, tablePower[0] = running kS
        public double startPower = 0.03;  // command offset needed to start from rest (stiction)
        public double delay = 0.04;       // s from command to the first change of speed
        public double accel = 1500;       // deg/s^2 toward a faster target speed
        public double decel = 1500;       // deg/s^2 back toward neutral
        public double loopDt = 0.017;
        public double loopJitter = 0.006;
        public double encoderNoise = 0.07; // deg, one sample
        public double velocityFilter = 0.3;
        public double hubFrame = 0.02;

        public Params copy() {
            Params p = new Params();
            p.tableSpeed = tableSpeed;
            p.tablePower = tablePower;
            p.startPower = startPower;
            p.delay = delay;
            p.accel = accel;
            p.decel = decel;
            p.loopDt = loopDt;
            p.loopJitter = loopJitter;
            p.encoderNoise = encoderNoise;
            p.velocityFilter = velocityFilter;
            p.hubFrame = hubFrame;
            return p;
        }
    }

    private static final double SUBSTEP = 0.002;
    private static final int PIPE = 64;

    private final Params p;
    private final Random rng;

    public double pos;
    public double vel;
    public double time;

    private double command = 0.5;
    private double applied = 0.5;
    private final double[] pipeTime = new double[PIPE];
    private final double[] pipeCmd = new double[PIPE];
    private int pipeHead = 0;
    private int pipeCount = 0;
    private double nextFrame;

    private double estPos;
    private double estVel;
    private double lastSampleTime;

    public TurretPlantModel(Params params, long seed) {
        this.p = params;
        this.rng = new Random(seed);
        this.nextFrame = rng.nextDouble() * p.hubFrame;
    }

    public void reset(double startPos) {
        pos = startPos;
        vel = 0;
        time = 0;
        command = 0.5;
        applied = 0.5;
        pipeHead = 0;
        pipeCount = 0;
        nextFrame = rng.nextDouble() * p.hubFrame;
        estPos = startPos;
        estVel = 0;
        lastSampleTime = 0;
    }

    public long nanos() {
        return (long) (time * 1e9);
    }

    // ---- SetTurretAngle.Sink
    @Override
    public void set(double position) {
        command = Math.max(0, Math.min(1, position));
    }

    // ---- SetTurretAngle.Sensor
    @Override
    public double getTurretAngle() {
        return estPos;
    }

    @Override
    public double getVelocity() {
        return estVel;
    }

    /** What the robot loop does at its start: read the encoder (noise, then the same velocity filter as AxonEncoder). */
    public void sample() {
        double measured = pos + rng.nextGaussian() * p.encoderNoise;
        double dt = time - lastSampleTime;
        if (dt > 0) {
            estVel += p.velocityFilter * ((measured - estPos) / dt - estVel);
        }
        estPos = measured;
        lastSampleTime = time;
    }

    /** One robot loop of simulated time. */
    public void advanceLoop() {
        advance(Math.max(0.006, p.loopDt + (rng.nextDouble() * 2 - 1) * p.loopJitter));
    }

    public void advance(double dt) {
        double end = time + dt;
        while (time < end - 1e-12) {
            double h = Math.min(SUBSTEP, end - time);
            while (time >= nextFrame) {
                push(nextFrame + p.delay, command);
                nextFrame += p.hubFrame;
            }
            while (pipeCount > 0 && pipeTime[(pipeHead - pipeCount + PIPE) % PIPE] <= time) {
                applied = pipeCmd[(pipeHead - pipeCount + PIPE) % PIPE];
                pipeCount--;
            }
            double offset = applied - 0.5;
            double target = targetSpeed(offset);
            if (Math.abs(vel) < 1.0 && Math.abs(offset) < p.startPower) {
                target = 0;
            }
            double rate = Math.abs(target) >= Math.abs(vel) && target * vel >= 0 ? p.accel : p.decel;
            if (target == 0 || target * vel < 0) {
                rate = p.decel;
            }
            double step = rate * h;
            double dv = target - vel;
            vel += Math.max(-step, Math.min(step, dv));
            pos += vel * h;
            time += h;
        }
    }

    private void push(double t, double c) {
        pipeHead = (pipeHead + 1) % PIPE;
        pipeTime[pipeHead] = t;
        pipeCmd[pipeHead] = c;
        pipeCount = Math.min(PIPE, pipeCount + 1);
    }

    /** Steady speed for a command offset, by interpolating the measured power -> speed table. */
    private double targetSpeed(double offset) {
        double a = Math.abs(offset);
        double[] pw = p.tablePower;
        double[] sp = p.tableSpeed;
        if (a <= pw[0]) {
            return 0;
        }
        int last = pw.length - 1;
        if (a >= pw[last]) {
            return Math.signum(offset) * sp[last];
        }
        int i = 1;
        while (i < last && a > pw[i]) {
            i++;
        }
        double f = (a - pw[i - 1]) / (pw[i] - pw[i - 1]);
        return Math.signum(offset) * (sp[i - 1] + f * (sp[i] - sp[i - 1]));
    }
}
