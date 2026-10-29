package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Finds ball centres in a yellow mask, including balls that touch or overlap.
 *
 * Balls rest on the floor, so a ball centred at pixel (u, v) has a KNOWN image radius from the
 * camera geometry alone. The detector covers the mask with discs of that radius, greedily:
 *
 *   score(p) = ( mask pixels in disc(p) - 0.8 * already-explained pixels - floorPenalty * floor pixels in disc )
 *              / disc area
 *
 * The best peak is taken, its disc marked explained, and the loop repeats, so a ball that is half
 * hidden behind a neighbour is judged only on the pixels the neighbour has not accounted for. Two
 * picks closer than sepFrac * 2R ON THE FLOOR are the same ball (balls cannot overlap on the floor).
 * Pure Java, no OpenCV: input is a byte mask (non-zero = yellow) of workWidth x workHeight.
 */
public final class ClusterDetector {

    public static final class Detection {
        public double u, v, radiusPx, score, coverage;
        public double xr, yr;                        // robot-frame floor position of the ball centre, cm (x fwd, y left)

        public double rangeCm() { return Math.hypot(xr, yr); }

        public double bearingDeg() { return Math.toDegrees(Math.atan2(yr, xr)); }
    }

    private final VisionParams p;
    private final BallCamera cam;
    private final int w, h;
    private final int[] rowK;
    private final int[][] hw;
    private final int[] kernelArea;
    private final float[] gx, gy, pixelR;
    private final int[] prefix;
    private final float[] mScore, ex, score;
    private final boolean[] blocked, explained;
    private final int[] candIdx;
    private int nCand;

    public ClusterDetector(BallCamera cam) {
        this.cam = cam;
        this.p = cam.p;
        this.w = cam.w;
        this.h = cam.h;
        rowK = new int[h];
        int maxK = 0;
        for (int v = 0; v < h; v++) {
            double r = cam.rowRadius[v] * p.radiusScale;
            rowK[v] = r >= p.minRadiusPx ? (int) Math.round(r) : 0;
            if (rowK[v] > maxK) maxK = rowK[v];
        }
        hw = new int[maxK + 1][];
        kernelArea = new int[maxK + 1];
        for (int k = 0; k <= maxK; k++) {
            hw[k] = new int[k + 1];
            int area = 0;
            for (int dy = 0; dy <= k; dy++) {
                hw[k][dy] = (int) Math.floor(Math.sqrt(k * k + 0.25 - dy * dy));
                area += (2 * hw[k][dy] + 1) * (dy == 0 ? 1 : 2);
            }
            kernelArea[k] = area;
        }
        gx = new float[w * h];
        gy = new float[w * h];
        pixelR = new float[w * h];
        for (int v = 0; v < h; v++) {
            for (int u = 0; u < w; u++) {
                double[] g = cam.groundPoint(u + 0.5, v + 0.5, p.ballRadiusCm);
                gx[v * w + u] = g == null ? 1e9f : (float) g[0];
                gy[v * w + u] = g == null ? 1e9f : (float) g[1];
                pixelR[v * w + u] = (float) cam.radiusPx(u + 0.5, v + 0.5);   // off-axis balls are bigger/smaller than the centre column's
            }
        }
        prefix = new int[(w + 1) * h];
        mScore = new float[w * h];
        ex = new float[w * h];
        score = new float[w * h];
        blocked = new boolean[w * h];
        explained = new boolean[w * h];
        candIdx = new int[w * h];
    }

    private int discSum(int v, int u, int k) {
        int s = 0;
        for (int dy = -k; dy <= k; dy++) {
            int row = v + dy;
            if (row < 0 || row >= h) continue;
            int half = hw[k][Math.abs(dy)];
            int uL = Math.max(u - half, 0), uR = Math.min(u + half, w - 1);
            s += prefix[row * (w + 1) + uR + 1] - prefix[row * (w + 1) + uL];
        }
        return s;
    }

    public List<Detection> detect(byte[] mask) {
        List<Detection> out = new ArrayList<>();
        // prefix sums of the mask, per row
        for (int v = 0; v < h; v++) {
            int base = v * (w + 1);
            prefix[base] = 0;
            for (int u = 0; u < w; u++) prefix[base + u + 1] = prefix[base + u] + (mask[v * w + u] != 0 ? 1 : 0);
        }
        Arrays.fill(ex, 0f);
        Arrays.fill(blocked, false);
        Arrays.fill(explained, false);
        nCand = 0;
        for (int v = 1; v < h - 1; v++) {
            if (rowK[v] < 2) continue;
            for (int u = 1; u < w - 1; u++) {
                int i = v * w + u;
                if (mask[i] != 0 && mask[i - 1] != 0 && mask[i + 1] != 0 && mask[i - w] != 0 && mask[i + w] != 0) {
                    candIdx[nCand++] = i;
                    mScore[i] = discSum(v, u, rowK[v]);
                }
            }
        }
        if (nCand == 0) return out;
        double sep2 = Math.pow(p.sepFrac * 2.0 * p.ballRadiusCm, 2);
        double[] camPos = cam.cameraPos();
        for (int iter = 0; iter < p.maxBalls; iter++) {
            float best = -9f;
            int bi = -1;
            for (int c = 0; c < nCand; c++) {
                int i = candIdx[c];
                if (blocked[i]) {
                    score[i] = -9f;
                    continue;
                }
                int v = i / w;
                float area = kernelArea[rowK[v]];
                float s = (mScore[i] - 0.8f * ex[i] - (float) p.floorPenalty * (area - mScore[i])) / area;
                score[i] = s;
                if (s > best) {
                    best = s;
                    bi = i;
                }
            }
            if (bi < 0 || best < p.scoreMin) break;
            int v0 = bi / w, u0 = bi % w;
            double r = pixelR[bi] * p.radiusScale;
            // sub-pixel: score-weighted centroid of the near-peak plateau
            int wn = Math.max(1, (int) Math.round(0.18 * r));
            double sw = 0, su = 0, sv = 0;
            for (int vv = Math.max(v0 - wn, 0); vv <= Math.min(v0 + wn, h - 1); vv++) {
                for (int uu = Math.max(u0 - wn, 0); uu <= Math.min(u0 + wn, w - 1); uu++) {
                    float sc = score[vv * w + uu];
                    if (sc < -5f) continue;
                    double wt = Math.max(sc - (best - 0.06), 0);
                    sw += wt;
                    su += wt * (uu + 0.5);
                    sv += wt * (vv + 0.5);
                }
            }
            double uc = sw > 0 ? su / sw : u0 + 0.5, vc = sw > 0 ? sv / sw : v0 + 0.5;
            double[] g = cam.groundPoint(uc, vc, p.ballRadiusCm);
            if (g == null) {
                blocked[bi] = true;
                continue;
            }
            for (int c = 0; c < nCand; c++) {            // same ball on the floor -> not a new pick
                int j = candIdx[c];
                double dx = gx[j] - g[0], dy = gy[j] - g[1];
                if (dx * dx + dy * dy < sep2) blocked[j] = true;
            }
            float area = kernelArea[rowK[v0]];
            double cov = mScore[bi] / area;
            double newFrac = (mScore[bi] - ex[bi]) / area;
            if (newFrac < p.newMin) continue;
            if (Math.hypot(g[0] - camPos[0], g[1] - camPos[1]) > p.maxRangeCm) continue;
            Detection d = new Detection();
            d.u = uc; d.v = vc; d.radiusPx = r; d.score = best; d.coverage = cov; d.xr = g[0]; d.yr = g[1];
            out.add(d);
            markExplained(mask, uc, vc, r);
        }
        return out;
    }

    /** Mark the disc's mask pixels explained and add them into every candidate's explained count. */
    private void markExplained(byte[] mask, double uc, double vc, double r) {
        int ri = (int) Math.ceil(r) + 1;
        int vcI = (int) vc, ucI = (int) uc;
        for (int vq = Math.max(vcI - ri, 0); vq <= Math.min(vcI + ri, h - 1); vq++) {
            for (int uq = Math.max(ucI - ri, 0); uq <= Math.min(ucI + ri, w - 1); uq++) {
                int iq = vq * w + uq;
                double dx = uq + 0.5 - uc, dy = vq + 0.5 - vc;
                if (dx * dx + dy * dy > r * r || mask[iq] == 0 || explained[iq]) continue;
                explained[iq] = true;
                int K = rowK[vq] + 3;
                for (int vp = Math.max(vq - K, 0); vp <= Math.min(vq + K, h - 1); vp++) {
                    int kp = rowK[vp];
                    if (kp < 2) continue;
                    for (int up = Math.max(uq - K, 0); up <= Math.min(uq + K, w - 1); up++) {
                        int ddx = up - uq, ddy = vp - vq;
                        if (ddx * ddx + ddy * ddy <= kp * kp + 0.25) ex[vp * w + up] += 1f;
                    }
                }
            }
        }
    }
}
