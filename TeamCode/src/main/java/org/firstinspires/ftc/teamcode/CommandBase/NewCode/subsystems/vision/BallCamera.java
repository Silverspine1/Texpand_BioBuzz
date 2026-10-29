package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.vision;

/**
 * Camera on the chassis: pixel <-> robot frame <-> field frame, with a wide-angle lens. Pure Java.
 * Robot frame: x forward, y LEFT, z up. Image: u right, v down.
 *
 * LENS: pixel radius from the image centre is  rho = f * g(theta),  g(theta) = (1-m)*theta + m*tan(theta),
 * theta = angle off the optical axis, m = lensRectMix. m = 0 is an EQUIDISTANT lens (angle proportional to pixel
 * radius - what the ELP-USB100W07M-MHV120 "FOV 120 H / 150 D" figures describe), m = 1 is a plain pinhole.
 * f is fixed by hfovDeg reaching the image edge. Entering 120 deg with the pinhole model (m = 1) makes balls look
 * ~40 % too small at the centre, which splits one ball into several detections.
 */
public final class BallCamera {

    public final VisionParams p;
    public final int w, h;
    public final double f, u0, v0;
    private final double[] fwd = new double[3], right = new double[3], down = new double[3], pos = new double[3];
    private static final double TH_MAX = 1.55;
    private static final int TAB = 2048;
    private final double[] thTab = new double[TAB], gTab = new double[TAB];
    /** Expected image radius (px) of a floor ball centred at each pixel; 0 where impossible. */
    public final float[] pixelRadius;

    public BallCamera(VisionParams p) {
        this(p, true);
    }

    /** tables = false skips the per-pixel ball-size table (used by the calibration fit, which needs only the geometry). */
    public BallCamera(VisionParams p, boolean tables) {
        this.p = p;
        this.w = p.workWidth;
        this.h = p.workHeight;
        for (int i = 0; i < TAB; i++) { thTab[i] = TH_MAX * i / (TAB - 1); gTab[i] = g(thTab[i]); }
        this.f = 0.5 * w / g(Math.toRadians(p.hfovDeg) / 2.0);
        this.u0 = 0.5 * w;
        this.v0 = 0.5 * h;
        double pitch = Math.toRadians(p.camPitchDownDeg);
        double yaw = Math.toRadians(p.camYawDeg);
        // pitch down about the robot y axis, then yaw (+ = left) about z
        double[] f0 = {Math.cos(pitch), 0, -Math.sin(pitch)};
        double[] r0 = {0, -1, 0};
        double[] d0 = {-Math.sin(pitch), 0, -Math.cos(pitch)};
        rot(f0, yaw, fwd);
        rot(r0, yaw, right);
        rot(d0, yaw, down);
        pos[0] = p.camForwardCm;
        pos[1] = p.camLeftCm;
        pos[2] = p.camHeightCm;
        pixelRadius = new float[w * h];
        if (tables)
            for (int v = 0; v < h; v++)
                for (int u = 0; u < w; u++) pixelRadius[v * w + u] = (float) radiusPx(u + 0.5, v + 0.5);
    }

    private double g(double th) {
        double m = p.lensRectMix;
        return (1.0 - m) * th + m * Math.tan(th);
    }

    /** Inverse of g (angle for a normalised radius), by table + linear interpolation. */
    private double gInv(double rho) {
        if (rho <= 0) return 0;
        if (rho >= gTab[TAB - 1]) return TH_MAX;
        int lo = 0, hi = TAB - 1;
        while (hi - lo > 1) {
            int mid = (lo + hi) >>> 1;
            if (gTab[mid] <= rho) lo = mid; else hi = mid;
        }
        double t = (rho - gTab[lo]) / Math.max(gTab[hi] - gTab[lo], 1e-12);
        return thTab[lo] + t * (thTab[hi] - thTab[lo]);
    }

    private static void rot(double[] a, double yaw, double[] out) {
        double c = Math.cos(yaw), s = Math.sin(yaw);
        out[0] = c * a[0] - s * a[1];
        out[1] = s * a[0] + c * a[1];
        out[2] = a[2];
    }

    public double[] cameraPos() {
        return pos.clone();
    }

    /** Robot-frame (x, y) where the ray through pixel (u, v) meets the plane z = zCm; null above the horizon. */
    public double[] groundPoint(double u, double v, double zCm) {
        double xc = (u - u0) / f, yc = (v - v0) / f;
        double rho = Math.hypot(xc, yc);
        double th = gInv(rho);
        double s = rho > 1e-9 ? Math.sin(th) / rho : 1.0;
        double c = Math.cos(th);
        double dx = fwd[0] * c + right[0] * xc * s + down[0] * yc * s;
        double dy = fwd[1] * c + right[1] * xc * s + down[1] * yc * s;
        double dz = fwd[2] * c + right[2] * xc * s + down[2] * yc * s;
        if (dz > -1e-6) return null;
        double t = (zCm - pos[2]) / dz;
        if (t <= 0) return null;
        return new double[]{pos[0] + t * dx, pos[1] + t * dy};
    }

    /** {u, v, depth} of a robot-frame point, or null if more than ~89 deg off the axis. */
    public double[] project(double x, double y, double z) {
        double px = x - pos[0], py = y - pos[1], pz = z - pos[2];
        double zc = px * fwd[0] + py * fwd[1] + pz * fwd[2];
        double xr = px * right[0] + py * right[1] + pz * right[2];
        double yd = px * down[0] + py * down[1] + pz * down[2];
        double lat = Math.hypot(xr, yd);
        double th = Math.atan2(lat, zc);
        if (th >= TH_MAX) return null;
        double k = lat > 1e-9 ? f * g(th) / lat : f;
        return new double[]{u0 + k * xr, v0 + k * yd, zc};
    }

    /** Image radius (px) of a ball of radius ballCm whose centre is at the robot-frame point (x, y, z). */
    public double ballRadiusPx(double x, double y, double z) {
        double cx = x - pos[0], cy = y - pos[1], cz = z - pos[2];
        double dist = Math.sqrt(cx * cx + cy * cy + cz * cz);
        double zc = cx * fwd[0] + cy * fwd[1] + cz * fwd[2];
        double xr = cx * right[0] + cy * right[1] + cz * right[2];
        double yd = cx * down[0] + cy * down[1] + cz * down[2];
        double th = Math.atan2(Math.hypot(xr, yd), zc);
        return radiusFrom(th, dist);
    }

    private double radiusFrom(double th, double dist) {
        double R = p.ballRadiusCm;
        double alpha = Math.asin(Math.min(R / Math.max(dist, R + 1e-6), 1.0));
        double radial = 0.5 * f * (g(Math.min(th + alpha, 1.5)) - g(Math.max(th - alpha, -1.5)));
        double tang = th < 1e-3 ? f * alpha : f * g(th) * Math.sin(alpha) / Math.max(Math.sin(th), 1e-3);
        return 0.5 * (radial + tang);
    }

    /** Expected image radius (px) of a floor ball whose centre projects to (u, v). 0 if impossible. */
    public double radiusPx(double u, double v) {
        double[] g0 = groundPoint(u, v, p.ballRadiusCm);
        if (g0 == null) return 0;
        double cx = g0[0] - pos[0], cy = g0[1] - pos[1], cz = p.ballRadiusCm - pos[2];
        double dist = Math.sqrt(cx * cx + cy * cy + cz * cz);
        double zc = cx * fwd[0] + cy * fwd[1] + cz * fwd[2];
        double xr = cx * right[0] + cy * right[1] + cz * right[2];
        double yd = cx * down[0] + cy * down[1] + cz * down[2];
        double th = Math.atan2(Math.hypot(xr, yd), zc);
        double alpha = Math.asin(Math.min(p.ballRadiusCm / Math.max(dist, p.ballRadiusCm + 1e-6), 1.0));
        if (th + alpha >= Math.toRadians(88)) return 0;
        return radiusFrom(th, dist);
    }

    // ---- robot <-> field (lateralSign handles the odometry frame's handedness) ----
    public double[] robotToField(double xr, double yr, double[] pose) {
        double hd = Math.toRadians(pose[2]), c = Math.cos(hd), s = Math.sin(hd);
        double ls = p.lateralSign;      // robot-left = ls * (-sin h, cos h)
        return new double[]{pose[0] + c * xr - ls * s * yr, pose[1] + s * xr + ls * c * yr};
    }

    public double[] fieldToRobot(double xf, double yf, double[] pose) {
        double hd = Math.toRadians(pose[2]), c = Math.cos(hd), s = Math.sin(hd);
        double dx = xf - pose[0], dy = yf - pose[1];
        double ls = p.lateralSign;
        return new double[]{c * dx + s * dy, ls * (-s * dx + c * dy)};
    }
}
