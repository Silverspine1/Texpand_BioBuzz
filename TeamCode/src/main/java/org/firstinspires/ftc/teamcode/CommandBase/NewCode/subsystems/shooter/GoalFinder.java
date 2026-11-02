package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

/**
 * Measures where the goal really is for AZIMUTH purposes. Stand at two (or more) spots well apart sideways, trim the turret until
 * the ball lands in the middle of the goal, and record each spot: the turret axis position (TEAM frame, inches) and the azimuth
 * the turret points at (TEAM frame, degrees). Each spot is a ray from the axis along that azimuth; the goal is the point nearest
 * to all rays (least squares). Pure Java, no allocation after construction.
 */
public final class GoalFinder {
    public static final int MAX = 6;
    /** Smallest allowed spread of the ray directions [deg]; closer than this the rays are parallel and the crossing is noise. */
    public static final double MIN_SPREAD_DEG = 12.0;

    private final double[] px = new double[MAX], py = new double[MAX], psi = new double[MAX];
    public int n;
    /** Result of the last successful {@link #solve}: the goal point (TEAM frame, in) and the RMS miss distance of the rays [in]. */
    public double goalX, goalY, rmsIn;

    public void clear() {
        n = 0;
    }

    /** @return false when the list is full */
    public boolean add(double pxIn, double pyIn, double psiDeg) {
        if (n >= MAX) return false;
        px[n] = pxIn;
        py[n] = pyIn;
        psi[n] = psiDeg;
        n++;
        return true;
    }

    /** Largest difference between two ray azimuths [deg] (0 with fewer than two). */
    public double spreadDeg() {
        double best = 0.0;
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                double d = Math.abs(((psi[i] - psi[j] + 540.0) % 360.0) - 180.0);
                if (d > best) best = d;
            }
        }
        return best;
    }

    /** Least-squares crossing of the rays. @return false with fewer than two spots or when the rays are nearly parallel */
    public boolean solve() {
        if (n < 2 || spreadDeg() < MIN_SPREAD_DEG) return false;
        double a11 = 0, a12 = 0, a22 = 0, b1 = 0, b2 = 0;
        for (int i = 0; i < n; i++) {
            double a = Math.toRadians(psi[i]);
            double nx = -Math.sin(a), ny = Math.cos(a);          // normal of the ray
            double d = nx * px[i] + ny * py[i];
            a11 += nx * nx;
            a12 += nx * ny;
            a22 += ny * ny;
            b1 += nx * d;
            b2 += ny * d;
        }
        double det = a11 * a22 - a12 * a12;
        if (Math.abs(det) < 1e-9) return false;
        goalX = (a22 * b1 - a12 * b2) / det;
        goalY = (a11 * b2 - a12 * b1) / det;
        double s = 0;
        for (int i = 0; i < n; i++) {
            double a = Math.toRadians(psi[i]);
            double e = -Math.sin(a) * (goalX - px[i]) + Math.cos(a) * (goalY - py[i]);
            s += e * e;
        }
        rmsIn = Math.sqrt(s / n);
        return true;
    }
}
