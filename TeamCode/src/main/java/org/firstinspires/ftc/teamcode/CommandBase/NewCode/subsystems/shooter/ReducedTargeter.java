package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

/**
 * The REDUCED shoot-while-moving targeter ("velocity-compensated aim table"), a line-by-line port of
 * {@code ftc_optimizer.shotcontrol.algos.reduced.solve_scalar} (spec: docs/SHOT_ALGO_DESIGN.md section 7).
 *
 * <p>Inputs and outputs are in the TEAM frame of docs/SHOT_CONTROL_PLAN.md: inches, seconds, degrees, origin at
 * the alliance's audience-side left field corner, +x forward (audience -> back wall), +y to the right seen from
 * the audience, headings and turret angles measured from +x toward +y. Converting the Java odometry pose into
 * that frame is {@link ShotFrame}'s job, not this class's.
 *
 * <p>Fixed cost, no allocation, no data-dependent loop counts: 2 muzzle passes, 3 + 1 inverse Newton steps,
 * up to 3 + 1 droop steps, bisection on each grid. Not thread-safe (scratch fields); use one instance per thread.
 * Pure Java (no FTC SDK).
 */
public final class ReducedTargeter {

    /** Output of one {@link #solve} call (overwritten in place). */
    public static final class Result {
        /** Wheel command [rpm] (includes the burst pre-compensation for queued balls). */
        public double rpmCmd;
        /** Wheel speed the shot model assumes at release [rpm] (= rpmCmd for the first ball of a burst). */
        public double rpmAtRelease;
        /** Hood command [deg of hood travel, 0..6.3] (servo angle = hood x 48/40). */
        public double hoodDeg;
        /** Turret angle relative to the robot heading [deg, team convention: + toward the robot's right]. */
        public double turretDeg;
        /** Claimed P(make), 0 when infeasible. */
        public double pMake;
        public boolean feasible;
        /** Flight time [s]. */
        public double tof;
        // diagnostics (same names as the Python debug dict)
        public double d, vr, vt, invErr, wRel, famErr, speed, angle;
        /** Field azimuth of the shot [deg, team frame] and the bearing pivot -> aim. */
        public double psiDeg, bearingDeg;
        /** Predicted wheel rpm drop of this shot (shot model "drop" field), for the shot detector. */
        public double dropRpm;
        /** Hive-occlusion factor applied to pMake (1 without an occ block) and the diagnostic P(make and stay)
         *  (NaN without an occ block). */
        public double occF, pStay;
        /** Part of turretDeg that is the model's bearing-rate lead (turret_lead_s x bearing rate) [deg]. */
        public double turretLeadDeg;
    }

    /** reduced.FALLBACK_VR_SCALES */
    static final double[] FALLBACK_VR_SCALES = {0.75, 0.5, 0.25, 0.0};

    private ReducedShotModel M;

    /** Driver trim of the exit speed [%], applied in exit space before the inverse (0 = Python parity). */
    public double speedTrimPct = 0.0;
    /** Field tuning: the real shooter launches this many degrees STEEPER than the model (exit space, before the inverse;
     *  0 = Python parity). Negative = flatter. */
    public double angleOffsetDeg = 0.0;
    /** > 0: the hive-occlusion map is the RED goal's; a goal on the other side of the field centre line is its mirror image, so the
     *  map is read at y -> occMirrorW - y (the field width). 0 = as exported. */
    public double occMirrorW = 0.0;
    /** > 0: the goal is the model cell's mirror image across the hive centre line (the FAR / scoring-side cell of the same hive), so the
     *  map is read at x -> occMirrorX - x (the field width). The aim table itself is symmetric in the line of sight, so nothing else changes.
     *  0 = as exported. */
    public double occMirrorX = 0.0;

    // scratch outputs of bicubic() / trilinear() (no allocation)
    private double bv, bdr, bdh;
    private double tSpeed, tAngle, tYaw, tTof, tP, tSlope, tRpm, tHood, tWok;
    private boolean tOk;
    // scratch of locate()
    private int li;
    private double lt, lh;

    public ReducedTargeter(ReducedShotModel model) {
        this.M = model;
    }

    public ReducedShotModel model() {
        return M;
    }

    /** Swap the model (e.g. after a field-tune reload). Not during a solve. */
    public void setModel(ReducedShotModel model) {
        this.M = model;
    }

    // ---------------------------------------------------------------------------------------------------------
    // helpers (reduced._locate, bicubic, trilinear, _bilinear, _wrap180)
    // ---------------------------------------------------------------------------------------------------------

    /** Sets li, lt, lh: index with g[i] <= x < g[i+1] (clamped), t in [0, 1], cell size. */
    private void locate(double[] g, double x) {
        int n = g.length;
        if (x <= g[0]) {
            li = 0; lt = 0.0; lh = g[1] - g[0];
            return;
        }
        if (x >= g[n - 1]) {
            li = n - 2; lt = 1.0; lh = g[n - 1] - g[n - 2];
            return;
        }
        int lo = 0, hi = n - 1;
        while (hi - lo > 1) {
            int mid = (lo + hi) >>> 1;
            if (g[mid] <= x) lo = mid;
            else hi = mid;
        }
        double h = g[lo + 1] - g[lo];
        li = lo; lt = (x - g[lo]) / h; lh = h;
    }

    /** Sets bv, bdr, bdh = value, d/drpm, d/dhood of shot field {@code f} at (rpm, hood). */
    private void bicubic(int f, double rpm, double hood) {
        locate(M.shotRpm, rpm);
        int i = li;
        double t = lt, hx = lh;
        locate(M.shotHood, hood);
        int j = li;
        double u = lt, hy = lh;
        double[] F = M.shot[f][0], Fx = M.shot[f][1], Fy = M.shot[f][2], Fxy = M.shot[f][3];
        int nH = M.nH;
        double t2 = t * t, t3 = t * t * t, u2 = u * u, u3 = u * u * u;
        double a0 = 2 * t3 - 3 * t2 + 1, a1 = t3 - 2 * t2 + t, a2 = -2 * t3 + 3 * t2, a3 = t3 - t2;
        double da0 = 6 * t2 - 6 * t, da1 = 3 * t2 - 4 * t + 1, da2 = -6 * t2 + 6 * t, da3 = 3 * t2 - 2 * t;
        double b0 = 2 * u3 - 3 * u2 + 1, b1 = u3 - 2 * u2 + u, b2 = -2 * u3 + 3 * u2, b3 = u3 - u2;
        double db0 = 6 * u2 - 6 * u, db1 = 3 * u2 - 4 * u + 1, db2 = -6 * u2 + 6 * u, db3 = 3 * u2 - 2 * u;
        double v = 0.0, dvx = 0.0, dvy = 0.0;
        // corners in the Python order: (0,0), (1,0), (0,1), (1,1) with basis pairs (a0,a1|a2,a3) x (b0,b1|b2,b3)
        for (int c = 0; c < 4; c++) {
            int di = c & 1, dj = c >> 1;
            double ax0 = di == 0 ? a0 : a2, ax1 = di == 0 ? a1 : a3;
            double dax0 = di == 0 ? da0 : da2, dax1 = di == 0 ? da1 : da3;
            double by0 = dj == 0 ? b0 : b2, by1 = dj == 0 ? b1 : b3;
            double dby0 = dj == 0 ? db0 : db2, dby1 = dj == 0 ? db1 : db3;
            int k = (i + di) * nH + (j + dj);
            double fv = F[k];
            double fx = Fx[k] * hx;
            double fy = Fy[k] * hy;
            double fxy = Fxy[k] * hx * hy;
            v += fv * ax0 * by0 + fx * ax1 * by0 + fy * ax0 * by1 + fxy * ax1 * by1;
            dvx += fv * dax0 * by0 + fx * dax1 * by0 + fy * dax0 * by1 + fxy * dax1 * by1;
            dvy += fv * ax0 * dby0 + fx * ax1 * dby0 + fy * ax0 * dby1 + fxy * ax1 * dby1;
        }
        bv = v;
        bdr = dvx / hx;
        bdh = dvy / hy;
    }

    /** Aim-table interpolation with the soft validity rule; sets t* fields. */
    private void trilinear(double d, double vr, double vtAbs) {
        locate(M.aimD, d);
        int i = li;
        double tx = lt;
        locate(M.aimVr, vr);
        int j = li;
        double ty = lt;
        locate(M.aimVt, vtAbs);
        int k = li;
        double tz = lt;
        double[] gd = M.aimD, gr = M.aimVr, gt = M.aimVt;
        boolean inside = (gd[0] <= d && d <= gd[M.nD - 1]) && (gr[0] <= vr && vr <= gr[M.nVr - 1]) && vtAbs <= gt[M.nVt - 1];
        int sI = M.nVr * M.nVt, sJ = M.nVt;
        double[] S = M.aim[ReducedShotModel.A_SPEED], A = M.aim[ReducedShotModel.A_ANGLE], Y = M.aim[ReducedShotModel.A_YAW],
                T = M.aim[ReducedShotModel.A_TOF], P = M.aim[ReducedShotModel.A_P], L = M.aim[ReducedShotModel.A_SLOPE],
                R = M.aim[ReducedShotModel.A_RPM], H = M.aim[ReducedShotModel.A_HOOD];
        boolean[] OK = M.aimOk;
        double wok = 0.0, s = 0.0, a = 0.0, y = 0.0, tf = 0.0, p = 0.0, sl = 0.0, r = 0.0, h = 0.0;
        for (int di = 0; di < 2; di++) {
            double wx = di == 1 ? tx : 1.0 - tx;
            for (int dj = 0; dj < 2; dj++) {
                double wy = dj == 1 ? ty : 1.0 - ty;
                for (int dk = 0; dk < 2; dk++) {
                    double wz = dk == 1 ? tz : 1.0 - tz;
                    int n = (i + di) * sI + (j + dj) * sJ + (k + dk);
                    if (!OK[n]) continue;
                    double w = wx * wy * wz;
                    wok += w;
                    s += w * S[n];
                    a += w * A[n];
                    y += w * Y[n];
                    tf += w * T[n];
                    p += w * P[n];
                    sl += w * L[n];
                    r += w * R[n];
                    h += w * H[n];
                }
            }
        }
        if (wok > 1e-12) {
            tSpeed = s / wok; tAngle = a / wok; tYaw = y / wok; tTof = tf / wok;
            tP = p; tSlope = sl / wok; tRpm = r / wok; tHood = h / wok;
        } else {
            // no valid corner: plain trilinear over all corners (infeasible nodes carry the nearest valid node's
            // values) so the tracking command stays continuous; flagged infeasible, claims P = 0
            s = 0.0; a = 0.0; y = 0.0; tf = 0.0; sl = 0.0; r = 0.0; h = 0.0;
            for (int di = 0; di < 2; di++) {
                double wx = di == 1 ? tx : 1.0 - tx;
                for (int dj = 0; dj < 2; dj++) {
                    double wy = dj == 1 ? ty : 1.0 - ty;
                    for (int dk = 0; dk < 2; dk++) {
                        double wz = dk == 1 ? tz : 1.0 - tz;
                        int n = (i + di) * sI + (j + dj) * sJ + (k + dk);
                        double w = wx * wy * wz;
                        s += w * S[n];
                        a += w * A[n];
                        y += w * Y[n];
                        tf += w * T[n];
                        sl += w * L[n];
                        r += w * R[n];
                        h += w * H[n];
                    }
                }
            }
            tSpeed = s; tAngle = a; tYaw = y; tTof = tf; tP = 0.0; tSlope = sl; tRpm = r; tHood = h;
        }
        tWok = wok;
        tOk = inside && wok >= M.okMin;
    }

    private double bilinear(double[] xs, double[] ys, double[] F, double x, double y) {
        locate(xs, x);
        int i = li;
        double t = lt;
        locate(ys, y);
        int j = li;
        double u = lt;
        int n1 = ys.length;
        return ((1 - t) * (1 - u) * F[i * n1 + j] + t * (1 - u) * F[(i + 1) * n1 + j]
                + (1 - t) * u * F[i * n1 + j + 1] + t * u * F[(i + 1) * n1 + j + 1]);
    }

    /** Python {@code (a + 180.0) % 360.0 - 180.0} (floor modulo). */
    static double wrap180(double a) {
        double r = (a + 180.0) % 360.0;
        if (r < 0) r += 360.0;
        return r - 180.0;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.min(Math.max(v, lo), hi);
    }

    // ---------------------------------------------------------------------------------------------------------
    // the algorithm (reduced.solve_scalar)
    // ---------------------------------------------------------------------------------------------------------

    /**
     * @param x,y          robot pose reference point [in, team frame] (estimate stamped pose_s before now)
     * @param headingDeg   [deg, from +x toward +y]
     * @param vx,vy        velocity [in/s]; ax, ay acceleration [in/s^2]; omegaDps yaw rate [deg/s, same sense]
     * @param shotIndex    0 = first ball of a burst (wheel settled); k = k balls already released in this burst
     * @param spacingS     time since the previous ball LEFT the shooter + fire_s [s] (ignored for index 0)
     * @param out          result, overwritten
     */
    public void solve(double x, double y, double headingDeg, double vx, double vy, double omegaDps,
                      double ax, double ay, int shotIndex, double spacingS, Result out) {
        final ReducedShotModel C = M;
        boolean hasMeas = shotIndex >= 1 && spacingS > 0.0;
        // 1. prediction to release
        double ta = C.poseS + C.fireS;
        double px = x + vx * ta + 0.5 * ax * ta * ta;
        double py = y + vy * ta + 0.5 * ay * ta * ta;
        double pvx = vx + ax * ta;
        double pvy = vy + ay * ta;
        double h = Math.toRadians(headingDeg + omegaDps * ta);
        double om = Math.toRadians(omegaDps);
        double ch = Math.cos(h), sn = Math.sin(h);
        // 2. turret pivot (robot fwd / right -> team: right = (-sin, cos))
        double ox = C.pivotFwdIn * ch - C.pivotRightIn * sn;
        double oy = C.pivotFwdIn * sn + C.pivotRightIn * ch;
        double qx = px + ox, qy = py + oy;
        double qvx = pvx - om * oy, qvy = pvy + om * ox;
        // 3. geometry to the aim point
        double rx = C.aimX - qx, ry = C.aimY - qy;
        double d = Math.hypot(rx, ry);
        double ux = rx / d, uy = ry / d;
        double arm = C.wheelFwdIn + C.armExtraIn;
        double bearing = Math.toDegrees(Math.atan2(ry, rx));
        double cl = ux, sl = uy;
        double vr = 0.0, vt = 0.0;
        for (int pass = 0; pass < C.nMuzzle; pass++) {
            double mvx = qvx - om * arm * sl;
            double mvy = qvy + om * arm * cl;
            vr = mvx * ux + mvy * uy;
            // robot-LEFT of the line of sight: in the LEFT-handed team frame left = (uy, -ux)
            vt = mvx * uy - mvy * ux;
            // 4. aim table
            trilinear(d, vr, Math.abs(vt));
            if (!tOk) {
                // infeasible: the table's yaw / speed at infeasible nodes is junk; pre-position at the nearest
                // feasible operating point instead (shrink the approach speed toward 0 until the table is valid);
                // the result stays flagged infeasible. If no scale is valid the original lookup is kept.
                double s0 = tSpeed, a0 = tAngle, y0 = tYaw, f0 = tTof, p0 = tP, l0 = tSlope, r0 = tRpm, h0 = tHood, w0 = tWok;
                boolean found = false;
                for (int fb = 0; fb < FALLBACK_VR_SCALES.length; fb++) {
                    trilinear(d, vr * FALLBACK_VR_SCALES[fb], Math.abs(vt));
                    if (tOk) {
                        tOk = false;
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    tSpeed = s0; tAngle = a0; tYaw = y0; tTof = f0; tP = p0; tSlope = l0; tRpm = r0; tHood = h0; tWok = w0;
                    tOk = false;
                }
            }
            double sgn = vt >= 0.0 ? 1.0 : -1.0;
            double psiR = Math.toRadians(bearing - sgn * tYaw);
            cl = Math.cos(psiR);
            sl = Math.sin(psiR);
        }
        boolean ok = tOk;
        double speed = tSpeed * (1.0 + 0.01 * speedTrimPct), angle = tAngle - angleOffsetDeg;
        // 5. inverse of the shot model (Newton, n_inv iterations)
        double rpm = tRpm, hood = tHood;
        double rLo = C.shotRpm[0], rHi = Math.min(C.shotRpm[C.nR - 1], C.rpmMax);
        double hLo = C.shotHood[0], hHi = C.shotHood[C.nH - 1];
        for (int it = 0; it < C.nInv; it++) {
            bicubic(ReducedShotModel.S_SPEED, rpm, hood);
            double s = bv, sR = bdr, sH = bdh;
            bicubic(ReducedShotModel.S_ANGLE, rpm, hood);
            double a = bv, aR = bdr, aH = bdh;
            double e1 = speed - s, e2 = angle - a;
            double det = sR * aH - sH * aR;
            if (Math.abs(det) < 1e-12) break;
            rpm = clamp(rpm + (aH * e1 - sH * e2) / det, rLo, rHi);
            hood = clamp(hood + (-aR * e1 + sR * e2) / det, hLo, hHi);
        }
        bicubic(ReducedShotModel.S_SPEED, rpm, hood);
        double sFin = bv;
        bicubic(ReducedShotModel.S_ANGLE, rpm, hood);
        double aFin = bv;
        double invErr = Math.abs(sFin - speed) / Math.max(0.01 * speed, 1e-6) + Math.abs(aFin - angle) / 0.5;
        // 6. droop (balls 2..n of a burst)
        double wRel = rpm;
        double famErr = 0.0;
        double rpmCmd = rpm;
        if (hasMeas) {
            double[] Dt = shotIndex == 1 ? C.deltaK1 : C.deltaKn;
            double[] Rt = shotIndex == 1 ? C.residK1 : C.residKn;
            rpmCmd = Math.min(rpm + bilinear(C.wheelRpm, C.wheelSpacing, Dt, rpm, spacingS), C.rpmMax);
            double e = bilinear(C.wheelRpm, C.wheelSpacing, Rt, rpm, spacingS);
            wRel = rpm + e;
            if (Math.abs(e) > C.droopMinRpm) {
                double hh = hood;
                for (int it = 0; it < C.nDroop; it++) {
                    bicubic(ReducedShotModel.S_SPEED, wRel, hh);
                    double s = bv, sH = bdh;
                    bicubic(ReducedShotModel.S_ANGLE, wRel, hh);
                    double a = bv, aH = bdh;
                    double f = s - speed - tSlope * (a - angle);
                    double df = sH - tSlope * aH;
                    if (Math.abs(df) < 1e-9) break;
                    hh = clamp(hh - f / df, hLo, hHi);
                }
                bicubic(ReducedShotModel.S_SPEED, wRel, hh);
                double s = bv;
                bicubic(ReducedShotModel.S_ANGLE, wRel, hh);
                double a = bv;
                famErr = s - speed - tSlope * (a - angle);
                if (Math.abs(famErr) < 2.0) hood = hh;
                else famErr = 0.0;          // no hood with authority: keep the set-point hood
            }
        }
        // 7. turret (table yaw is + toward the robot's LEFT; team-frame angles grow toward +y = right)
        double sgn = vt >= 0.0 ? 1.0 : -1.0;
        double psi = bearing - sgn * tYaw;
        double turret = wrap180(psi - Math.toDegrees(h));
        double bRate = Math.toDegrees((rx * (-qvy) - ry * (-qvx)) / (d * d));
        turret = turret + C.turretLeadS * bRate;
        out.turretLeadDeg = C.turretLeadS * bRate;
        // 8. claim
        boolean feasible = ok && invErr < 4.0;
        double p = feasible ? tP : 0.0;
        out.occF = 1.0;
        out.pStay = Double.NaN;
        if (C.hasOcc) {
            // hive occlusion (position-dependent, the aim table cannot see it): un-blocked fraction at the pivot
            double qyOcc = occMirrorW > 0.0 ? occMirrorW - qy : qy;
            double qxOcc = occMirrorX > 0.0 ? occMirrorX - qx : qx;
            double fOcc = bilinear(C.occX, C.occY, C.occF, qxOcc, qyOcc);
            double sStay = bilinear(C.occX, C.occY, C.occStay, qxOcc, qyOcc);
            out.occF = fOcc;
            out.pStay = p * fOcc * sStay;
            p = p * fOcc;
        }
        out.rpmCmd = rpmCmd;
        out.rpmAtRelease = wRel;
        out.hoodDeg = hood;
        out.turretDeg = turret;
        out.pMake = p;
        out.feasible = feasible;
        out.tof = tTof;
        out.d = d;
        out.vr = vr;
        out.vt = vt;
        out.invErr = invErr;
        out.wRel = wRel;
        out.famErr = famErr;
        out.speed = speed;
        out.angle = angle;
        out.psiDeg = psi;
        out.bearingDeg = bearing;
        bicubic(ReducedShotModel.S_DROP, rpm, hood);
        out.dropRpm = bv;
    }
}
