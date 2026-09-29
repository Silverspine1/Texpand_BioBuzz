package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

import java.util.Arrays;

/**
 * Grid of field positions the shot planner can (or can't) make a shot from, for one goal.
 * Computed on a background thread and recomputed whenever the planner settings or goal change.
 */
public class ReachabilityMap {

    public static final byte NONE = 0;
    public static final byte SHOT = 1;
    public static final byte SHOT_TIGHT = 2;

    public static final class Snapshot {
        public final double cell;
        public final int size;
        public final byte[] state;
        final long signature;

        Snapshot(double cell, int size, byte[] state, long signature) {
            this.cell = cell;
            this.size = size;
            this.state = state;
            this.signature = signature;
        }

        public byte at(int ix, int iy) {
            return state[iy * size + ix];
        }
    }

    private final double fieldSize;
    private volatile Snapshot latest;
    private volatile long wanted;
    private volatile boolean computing;

    public ReachabilityMap(double fieldSizeMeters) {
        this.fieldSize = fieldSizeMeters;
    }

    public boolean isComputing() {
        return computing;
    }

    /** Cheap to call every loop. Returns the most recent finished map, or null before the first one completes. */
    public Snapshot update(Shotplanner.Config config, Shotplanner.HiveTarget hive, double cellMeters, double delaySeconds) {
        double cell = Math.max(0.1, cellMeters);
        long signature = signature(config, hive, cell, delaySeconds);
        wanted = signature;

        Snapshot current = latest;
        if ((current == null || current.signature != signature) && !computing) {
            computing = true;
            Shotplanner.Config configCopy = config.copy();
            Thread worker = new Thread(() -> compute(configCopy, hive, cell, delaySeconds, signature), "reachability-map");
            worker.setDaemon(true);
            worker.setPriority(Thread.MIN_PRIORITY);
            worker.start();
        }
        return latest;
    }

    private void compute(Shotplanner.Config config, Shotplanner.HiveTarget hive, double cell, double delay, long signature) {
        try {
            Shotplanner planner = new Shotplanner(config);
            int size = (int) Math.ceil(fieldSize / cell);
            byte[] state = new byte[size * size];

            for (int iy = 0; iy < size; iy++) {
                if (wanted != signature) return;
                for (int ix = 0; ix < size; ix++) {
                    double x = (ix + 0.5) * cell;
                    double y = (iy + 0.5) * cell;
                    double heading = Math.toDegrees(Math.atan2(hive.openingY - y, hive.openingX - x));

                    Shotplanner.RobotState robot = new Shotplanner.RobotState(x, y, 0, 0, heading, 0, 0);
                    Shotplanner.ShotSolution shot = planner.calculateShot(robot, hive, delay);
                    state[iy * size + ix] = !shot.reachable ? NONE : shot.safe ? SHOT : SHOT_TIGHT;
                }
            }
            latest = new Snapshot(cell, size, state, signature);
        } catch (RuntimeException ignored) {
        } finally {
            computing = false;
        }
    }

    private static long signature(Shotplanner.Config c, Shotplanner.HiveTarget h, double cell, double delay) {
        return Arrays.hashCode(new double[]{
                c.pollenDiameterMeters, c.flywheelRadiusMeters, c.muzzleEfficiency, c.turretAxisForwardMeters,
                c.turretAxisLeftMeters, c.muzzleForwardFromTurretMeters, c.muzzleLeftFromTurretMeters,
                c.muzzleHeightMeters, c.minLaunchAngleDeg, c.maxLaunchAngleDeg, c.maxFlywheelRPM, c.magnusK,
                c.requiredClearanceMeters, c.cellWidthMeters, c.cellVerticalSideHeightMeters, c.cellTotalHeightMeters,
                c.entryCheckDepthMeters, h.openingX, h.openingY, h.openingBottomZ, h.facingAngleDeg, cell, delay
        });
    }
}
