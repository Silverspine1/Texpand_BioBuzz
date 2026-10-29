package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.vision;

/**
 * Pinhole camera on the chassis: pixel <-> robot frame <-> field frame.
 * Robot frame: x forward, y LEFT, z up. Image: u right, v down. Pure Java.
 */
public final class BallCamera {

    public final VisionParams p;
    public final int w, h;
    public final double f, u0, v0;
    private final double[] fwd = new double[3], right = new double[3], down = new double[3], pos = new double[3];
    public final double[] rowRadius;       // expected ball radius (px) per row, centre column; 0 = none

    public BallCamera(VisionParams p) {
        this.p = p;
        this.w = p.workWidth;
        this.h = p.workHeight;
        this.f = 0.5 * w / Math.tan(Math.toRadians(p.hfovDeg) / 2.0);
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
        rowRadius = new double[h];
        for (int v = 0; v < h; v++) rowRadius[v] = radiusPx(u0, v + 0.5);
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
        double dx = fwd[0] + right[0] * xc + down[0] * yc;
        double dy = fwd[1] + right[1] * xc + down[1] * yc;
        double dz = fwd[2] + right[2] * xc + down[2] * yc;
        double n = Math.sqrt(dx * dx + dy * dy + dz * dz);
        dx /= n; dy /= n; dz /= n;
        if (dz > -1e-6) return null;
        double t = (zCm - pos[2]) / dz;
        if (t <= 0) return null;
        return new double[]{pos[0] + t * dx, pos[1] + t * dy};
    }

    /** {u, v, depth} of a robot-frame point, or null if behind the camera. */
    public double[] project(double x, double y, double z) {
        double px = x - pos[0], py = y - pos[1], pz = z - pos[2];
        double zc = px * fwd[0] + py * fwd[1] + pz * fwd[2];
        if (zc < 1e-3) return null;
        double xr = px * right[0] + py * right[1] + pz * right[2];
        double yd = px * down[0] + py * down[1] + pz * down[2];
        return new double[]{u0 + f * xr / zc, v0 + f * yd / zc, zc};
    }

    /** Expected image radius (px) of a floor ball whose centre projects to (u, v). 0 if impossible. */
    public double radiusPx(double u, double v) {
        double[] g = groundPoint(u, v, p.ballRadiusCm);
        if (g == null) return 0;
        double cx = g[0] - pos[0], cy = g[1] - pos[1], cz = p.ballRadiusCm - pos[2];
        double dist = Math.sqrt(cx * cx + cy * cy + cz * cz);
        double alpha = Math.asin(Math.min(p.ballRadiusCm / Math.max(dist, p.ballRadiusCm + 1e-6), 1.0));
        double zc = cx * fwd[0] + cy * fwd[1] + cz * fwd[2];
        double theta = Math.acos(Math.max(-1, Math.min(1, zc / Math.max(dist, 1e-9))));
        if (theta + alpha >= Math.toRadians(88)) return 0;
        double radial = 0.5 * f * (Math.tan(Math.min(theta + alpha, 1.5)) - Math.tan(Math.max(theta - alpha, -1.5)));
        double tang = f * Math.tan(alpha) / Math.max(Math.cos(theta), 0.05);
        return 0.5 * (radial + tang);
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
