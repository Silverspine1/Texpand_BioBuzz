package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Finds ball centres in a yellow mask, including balls that touch or overlap. Pure Java, no OpenCV:
 * input is a byte mask (non-zero = yellow) of workWidth x workHeight.
 *
 * Balls rest on the floor, so a ball centred at pixel (u, v) has a KNOWN image radius from the camera geometry
 * and lens alone (BallCamera.pixelRadius - per pixel, because with a wide-angle lens the size changes across the
 * row as well as down it). The detector covers the mask with discs of that radius, greedily:
 *
 *   score(p) = ( mask pixels in disc(p) - 0.8 * already-explained pixels - floorPenalty * floor pixels in disc )
 *              / disc area
 *
 * The best peak is taken, its disc marked explained, repeat - so a ball half hidden behind a neighbour is judged
 * only on the pixels the neighbour has not already accounted for. While picking, only DUPLICATES of a pick (floor
 * centres closer than dupFrac * 2R) are blocked.
 *
 * Impossible-overlap filter: two balls cannot overlap on the floor, so centres closer than sepFrac * 2R cannot both
 * be real (same rule in the image: closer than imgSepFrac * (r1 + r2), which still works when the lens model is
 * wrong). For every conflicting pair the one with the smaller UNIQUE SUPPORT (mask pixels in its disc that no other
 * detection explains) is dropped. The classic false detection - a "ghost" between two touching balls, which scores
 * like a ball because the mask there is fully covered - has almost no unique support, the real balls a lot.
 */
public final class ClusterDetector {

    public static final class Detection {
        public double u, v, radiusPx, score, coverage, unique = 1.0;
        public double xr, yr;                        // robot-frame floor position of the ball centre, cm (x fwd, y left)

        public double rangeCm() { return Math.hypot(xr, yr); }

        public double bearingDeg() { return Math.toDegrees(Math.atan2(yr, xr)); }
    }

    private final VisionParams p;
    private final BallCamera cam;
    private final int w, h;
    private final int[] kmap;                         // disc radius per pixel (0 = none)
    private final int[][] hw;
    private final int[] kArea;
    private final float[] gx, gy;
    private final int[] prefM;                        // row prefix sums of the mask
    private final int[] prefEx;                       // row prefix sums of the explained pixels
    private final byte[] explained;
    private final int[] cnt;
    private int[] cv, cu, ck;
    private float[] cM, cArea, cEx, cScore, cgx, cgy;
    private boolean[] alive;

    public ClusterDetector(BallCamera cam) {
        this.cam = cam;
        this.p = cam.p;
        this.w = cam.w;
        this.h = cam.h;
        kmap = new int[w * h];
        int maxK = 0;
        for (int i = 0; i < w * h; i++) {
            double r = cam.pixelRadius[i] * p.radiusScale;
            kmap[i] = r >= p.minRadiusPx ? (int) Math.round(r) : 0;
            if (kmap[i] > maxK) maxK = kmap[i];
        }
        hw = new int[maxK + 1][];
        kArea = new int[maxK + 1];
        for (int k = 0; k <= maxK; k++) {
            hw[k] = new int[k + 1];
            int area = 0;
            for (int dy = 0; dy <= k; dy++) {
                hw[k][dy] = (int) Math.floor(Math.sqrt(k * k + 0.25 - dy * dy));
                area += (2 * hw[k][dy] + 1) * (dy == 0 ? 1 : 2);
            }
            kArea[k] = area;
        }
        gx = new float[w * h];
        gy = new float[w * h];
        for (int v = 0; v < h; v++) {
            for (int u = 0; u < w; u++) {
                double[] g = cam.groundPoint(u + 0.5, v + 0.5, p.ballRadiusCm);
                gx[v * w + u] = g == null ? 1e9f : (float) g[0];
                gy[v * w + u] = g == null ? 1e9f : (float) g[1];
            }
        }
        prefM = new int[(w + 1) * h];
        prefEx = new int[(w + 1) * h];
        explained = new byte[w * h];
        cnt = new int[w * h];
    }

    private static int disc(int[] pref, int w, int h, int[][] hw, int v, int u, int k) {
        int s = 0;
        for (int dy = -k; dy <= k; dy++) {
            int row = v + dy;
            if (row < 0 || row >= h) continue;
            int half = hw[k][Math.abs(dy)];
            int uL = Math.max(u - half, 0), uR = Math.min(u + half, w - 1);
            s += pref[row * (w + 1) + uR + 1] - pref[row * (w + 1) + uL];
        }
        return s;
    }

    private void rebuildRow(int[] pref, byte[] src, int row, boolean isMask, byte[] mask) {
        int base = row * (w + 1);
        pref[base] = 0;
        for (int u = 0; u < w; u++) {
            int on = isMask ? (mask[row * w + u] != 0 ? 1 : 0) : src[row * w + u];
            pref[base + u + 1] = pref[base + u] + on;
        }
    }

    public List<Detection> detect(byte[] mask) {
        List<Detection> picks = new ArrayList<>();
        for (int v = 0; v < h; v++) rebuildRow(prefM, null, v, true, mask);
        Arrays.fill(explained, (byte) 0);
        Arrays.fill(prefEx, 0);
        // candidates: eroded mask pixels that have a disc size
        int n = 0;
        int[] tmpIdx = new int[w * h];
        for (int v = 1; v < h - 1; v++) {
            for (int u = 1; u < w - 1; u++) {
                int i = v * w + u;
                if (kmap[i] >= 2 && mask[i] != 0 && mask[i - 1] != 0 && mask[i + 1] != 0 && mask[i - w] != 0 && mask[i + w] != 0)
                    tmpIdx[n++] = i;
            }
        }
        if (n == 0) return picks;
        cv = new int[n]; cu = new int[n]; ck = new int[n];
        cM = new float[n]; cArea = new float[n]; cEx = new float[n]; cScore = new float[n];
        cgx = new float[n]; cgy = new float[n]; alive = new boolean[n];
        for (int c = 0; c < n; c++) {
            int i = tmpIdx[c];
            cv[c] = i / w; cu[c] = i % w; ck[c] = kmap[i];
            cArea[c] = kArea[ck[c]];
            cM[c] = disc(prefM, w, h, hw, cv[c], cu[c], ck[c]);
            cScore[c] = (cM[c] - (float) p.floorPenalty * (cArea[c] - cM[c])) / cArea[c];
            cgx[c] = gx[i]; cgy[c] = gy[i];
            alive[c] = true;
        }
        double dup2 = Math.pow(p.dupFrac * 2.0 * p.ballRadiusCm, 2);
        double[] camPos = cam.cameraPos();
        for (int iter = 0; iter < p.maxBalls; iter++) {
            float best = -9f;
            int bi = -1;
            for (int c = 0; c < n; c++) if (alive[c] && cScore[c] > best) { best = cScore[c]; bi = c; }
            if (bi < 0 || best < p.scoreMin) break;
            int v0 = cv[bi], u0 = cu[bi], k = ck[bi];
            double r = k;
            // sub-pixel: score-weighted centroid of the near-peak plateau
            int wn = Math.max(1, (int) Math.round(0.18 * r));
            double sw = 0, su = 0, sv = 0;
            for (int c = 0; c < n; c++) {
                if (!alive[c] || Math.abs(cv[c] - v0) > wn || Math.abs(cu[c] - u0) > wn) continue;
                double wt = Math.max(cScore[c] - (best - 0.06), 0);
                sw += wt; su += wt * (cu[c] + 0.5); sv += wt * (cv[c] + 0.5);
            }
            double uc = sw > 0 ? su / sw : u0 + 0.5, vc = sw > 0 ? sv / sw : v0 + 0.5;
            double[] g = cam.groundPoint(uc, vc, p.ballRadiusCm);
            alive[bi] = false;
            if (g == null) continue;
            for (int c = 0; c < n; c++) {                 // duplicates of this ball
                double dx = cgx[c] - g[0], dy = cgy[c] - g[1];
                if (dx * dx + dy * dy < dup2) alive[c] = false;
            }
            double cov = cM[bi] / cArea[bi];
            if ((cM[bi] - cEx[bi]) / cArea[bi] < p.newMin) continue;
            if (Math.hypot(g[0] - camPos[0], g[1] - camPos[1]) > p.maxRangeCm) continue;
            Detection d = new Detection();
            d.u = uc; d.v = vc; d.radiusPx = r; d.score = best; d.coverage = cov; d.xr = g[0]; d.yr = g[1];
            picks.add(d);
            // mark the disc explained, refresh the explained-count of every candidate it can touch
            int ri = k + 1;
            int va = Math.max(v0 - ri, 0), vb = Math.min(v0 + ri, h - 1);
            for (int vq = va; vq <= vb; vq++) {
                for (int uq = Math.max(u0 - ri, 0); uq <= Math.min(u0 + ri, w - 1); uq++) {
                    double ddx = uq + 0.5 - uc, ddy = vq + 0.5 - vc;
                    if (ddx * ddx + ddy * ddy <= r * r && mask[vq * w + uq] != 0) explained[vq * w + uq] = 1;
                }
                rebuildRow(prefEx, explained, vq, false, null);
            }
            for (int c = 0; c < n; c++) {
                if (!alive[c] || Math.hypot(cv[c] - v0, cu[c] - u0) > r + ck[c] + 1.5) continue;
                cEx[c] = disc(prefEx, w, h, hw, cv[c], cu[c], ck[c]);
                cScore[c] = (cM[c] - 0.8f * cEx[c] - (float) p.floorPenalty * (cArea[c] - cM[c])) / cArea[c];
            }
        }
        return filterOverlaps(picks, mask);
    }

    private double[] uniqueSupport(List<Detection> dets, byte[] mask) {
        Arrays.fill(cnt, 0);
        for (Detection d : dets) paintDisc(d, mask, 1, null);
        double[] out = new double[dets.size()];
        for (int i = 0; i < out.length; i++) {
            Detection d = dets.get(i);
            int[] acc = new int[1];
            paintDisc(d, mask, 0, acc);
            out[i] = acc[0] / Math.max(Math.PI * d.radiusPx * d.radiusPx, 1.0);
        }
        return out;
    }

    /** mode 1: add the disc's mask pixels to cnt; mode 0: count the disc's mask pixels whose cnt == 1 into acc[0]. */
    private void paintDisc(Detection d, byte[] mask, int mode, int[] acc) {
        double r = d.radiusPx;
        int va = Math.max((int) (d.v - r) - 1, 0), vb = Math.min((int) (d.v + r) + 1, h - 1);
        int ua = Math.max((int) (d.u - r) - 1, 0), ub = Math.min((int) (d.u + r) + 1, w - 1);
        for (int v = va; v <= vb; v++) {
            for (int u = ua; u <= ub; u++) {
                double dx = u + 0.5 - d.u, dy = v + 0.5 - d.v;
                if (dx * dx + dy * dy > r * r || mask[v * w + u] == 0) continue;
                if (mode == 1) cnt[v * w + u]++;
                else if (cnt[v * w + u] == 1) acc[0]++;
            }
        }
    }

    private List<Detection> filterOverlaps(List<Detection> in, byte[] mask) {
        List<Detection> dets = new ArrayList<>(in);
        double sep2 = Math.pow(p.sepFrac * 2.0 * p.ballRadiusCm, 2);
        while (dets.size() > 1) {
            double[] uq = uniqueSupport(dets, mask);
            int worst = -1;
            double wu = 0;
            for (int i = 0; i < dets.size(); i++) {
                for (int j = i + 1; j < dets.size(); j++) {
                    Detection a = dets.get(i), b = dets.get(j);
                    double dxy = (a.xr - b.xr) * (a.xr - b.xr) + (a.yr - b.yr) * (a.yr - b.yr);
                    double di = Math.hypot(a.u - b.u, a.v - b.v);
                    if (dxy < sep2 || di < p.imgSepFrac * (a.radiusPx + b.radiusPx)) {
                        boolean iLoses = uq[i] < uq[j] || (uq[i] == uq[j] && a.score < b.score);
                        int loser = iLoses ? i : j;
                        if (worst < 0 || uq[loser] < wu) { worst = loser; wu = uq[loser]; }
                    }
                }
            }
            if (worst < 0) break;
            dets.remove(worst);
        }
        double[] uq = uniqueSupport(dets, mask);
        List<Detection> out = new ArrayList<>();
        for (int i = 0; i < dets.size(); i++) {
            dets.get(i).unique = uq[i];
            if (uq[i] >= p.uniqueMin) out.add(dets.get(i));
        }
        return out;
    }
}
