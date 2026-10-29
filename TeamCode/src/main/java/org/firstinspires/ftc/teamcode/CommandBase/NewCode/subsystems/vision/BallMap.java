package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.vision;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Global ball map with short-term memory.
 *
 * Detections arrive in the ROBOT frame and are moved to the FIELD frame with the pose the robot had
 * when the frame was EXPOSED (pose history, interpolated), so the robot's own motion during the
 * pipeline delay cancels out. A ball that keeps landing on the same field spot is one track; its
 * position is a running static-target estimate (more frames = more certain, close views dominate
 * because their measurement sigma is small).
 *
 * Memory: a track that leaves the picture is still planned against for memoryS. It is dropped
 * earlier when it should be visible but is repeatedly not detected (ball gone), or the intake
 * captured a ball at the mouth. Nothing here predicts motion: balls are assumed static.
 */
public final class BallMap {

    public static final class Track {
        public final int id;
        public double x, y, var;
        public int hits = 1, misses = 0;
        public double lastSeenS;
        boolean seenNow;

        Track(int id, double x, double y, double var, double t) {
            this.id = id; this.x = x; this.y = y; this.var = var; this.lastSeenS = t;
        }

        public double sigma() { return Math.sqrt(var); }
    }

    private VisionParams p;
    private final BallCamera cam;
    private final Map<Integer, Track> tracks = new LinkedHashMap<>();
    private int nextId = 0;
    private final ArrayDeque<double[]> hist = new ArrayDeque<>();   // {t, x, y, heading}
    public int created, droppedUnseen, droppedMemory, droppedCaptured;

    public BallMap(BallCamera cam) {
        this.cam = cam;
        this.p = cam.p;
    }

    /** Tracker/planner tunables that changed without the camera geometry changing. */
    public void setParams(VisionParams v) { this.p = v; }

    public List<Track> all() { return new ArrayList<>(tracks.values()); }

    public void clear() { tracks.clear(); }

    public void remove(int id) { tracks.remove(id); }

    public int size() { return tracks.size(); }

    // ---- pose history ----
    public void pushPose(double tS, double x, double y, double headingDeg) {
        hist.addLast(new double[]{tS, x, y, headingDeg});
        while (!hist.isEmpty() && hist.peekFirst()[0] < tS - 1.5) hist.pollFirst();
    }

    /** {x, y, heading} at time tS (clamped to the history), or null if empty. */
    public double[] poseAt(double tS) {
        if (hist.isEmpty()) return null;
        double[] prev = null;
        for (double[] cur : hist) {
            if (cur[0] >= tS) {
                if (prev == null) return new double[]{cur[1], cur[2], cur[3]};
                double f = (tS - prev[0]) / Math.max(cur[0] - prev[0], 1e-9);
                double dh = ((cur[3] - prev[3] + 540.0) % 360.0) - 180.0;
                return new double[]{prev[1] + f * (cur[1] - prev[1]), prev[2] + f * (cur[2] - prev[2]), prev[3] + f * dh};
            }
            prev = cur;
        }
        return new double[]{prev[1], prev[2], prev[3]};
    }

    public double measSigma(double rangeCm) {
        double s = p.measSigma0Cm + p.measSigmaPerCm * Math.max(rangeCm - 60.0, 0.0);
        return Math.sqrt(s * s + p.poseSigmaCm * p.poseSigmaCm
                + Math.pow(p.calibK * rangeCm * rangeCm, 2));
    }

    /** Would a floor ball at field (x, y) be inside the image from {@code pose}? */
    public boolean inView(double x, double y, double[] pose) {
        double[] r = cam.fieldToRobot(x, y, pose);
        double[] cp = cam.cameraPos();
        double rng = Math.hypot(r[0] - cp[0], r[1] - cp[1]);
        if (rng < p.minViewRangeCm || rng > p.maxViewRangeCm) return false;
        double[] uv = cam.project(r[0], r[1], p.ballRadiusCm);
        if (uv == null) return false;
        double m = p.viewMarginPx;
        return uv[0] >= m && uv[0] <= cam.w - m && uv[1] >= m && uv[1] <= cam.h - m;
    }

    /** Fold one frame in. dets are robot-frame floor positions {xr, yr}, seen at tExposeS. */
    public void update(double tExposeS, List<ClusterDetector.Detection> dets, double tNowS) {
        double[] pose = poseAt(tExposeS);
        if (pose == null) return;
        for (Track t : tracks.values()) t.seenNow = false;
        double lx = p.fieldHalfXCm - p.ballRadiusCm, ly = p.fieldHalfYCm - p.ballRadiusCm;
        List<double[]> meas = new ArrayList<>();      // {x, y, var}
        double[] cp = cam.cameraPos();
        for (ClusterDetector.Detection d : dets) {
            double rng = Math.hypot(d.xr - cp[0], d.yr - cp[1]);
            if (rng > p.maxDetRangeCm) continue;
            double[] f = cam.robotToField(d.xr, d.yr, pose);
            if (Math.abs(f[0]) > lx + p.outsideRejectCm || Math.abs(f[1]) > ly + p.outsideRejectCm) continue;   // beyond the wall: a ranging error
            double s = measSigma(rng);
            meas.add(new double[]{Math.max(-lx, Math.min(lx, f[0])), Math.max(-ly, Math.min(ly, f[1])), s * s});
        }
        // greedy nearest association
        List<double[]> pairs = new ArrayList<>();     // {dist, meas index, track id}
        for (int j = 0; j < meas.size(); j++) {
            double[] m = meas.get(j);
            for (Track t : tracks.values()) {
                double d = Math.hypot(m[0] - t.x, m[1] - t.y);
                double gate = p.gateCm + 2.0 * Math.sqrt(t.var + m[2]);
                if (d <= gate) pairs.add(new double[]{d, j, t.id});
            }
        }
        pairs.sort((a, b) -> Double.compare(a[0], b[0]));
        boolean[] usedM = new boolean[meas.size()];
        java.util.HashSet<Integer> usedT = new java.util.HashSet<>();
        for (double[] pr : pairs) {
            int j = (int) pr[1], tid = (int) pr[2];
            if (usedM[j] || usedT.contains(tid)) continue;
            usedM[j] = true;
            usedT.add(tid);
            Track t = tracks.get(tid);
            double[] m = meas.get(j);
            double k = t.var / (t.var + m[2]);
            t.x += k * (m[0] - t.x);
            t.y += k * (m[1] - t.y);
            t.var = (1.0 - k) * t.var;
            t.hits++;
            t.misses = 0;
            t.lastSeenS = tNowS;
            t.seenNow = true;
        }
        for (int j = 0; j < meas.size(); j++) {
            if (usedM[j]) continue;
            double[] m = meas.get(j);
            Track t = new Track(nextId, m[0], m[1], m[2], tNowS);
            t.seenNow = true;
            tracks.put(nextId++, t);
            created++;
        }
        // misses: expected in view now but not seen
        for (Track t : new ArrayList<>(tracks.values())) {
            if (t.seenNow) continue;
            if (inView(t.x, t.y, pose)) {
                t.misses++;
                if (t.misses >= p.missDrop) {
                    tracks.remove(t.id);
                    droppedUnseen++;
                    continue;
                }
            }
            if (tNowS - t.lastSeenS > p.memoryS) {
                tracks.remove(t.id);
                droppedMemory++;
            }
        }
    }

    public void pruneAge(double tNowS) {
        for (Track t : new ArrayList<>(tracks.values())) {
            if (tNowS - t.lastSeenS > p.memoryS) {
                tracks.remove(t.id);
                droppedMemory++;
            }
        }
    }

    /** The intake took a ball: remove the track nearest the mouth. */
    public void onCapture(double[] pose) {
        double[] m = cam.robotToField(p.intakeForwardCm * 0.3, 0.0, pose);
        Track best = null;
        double bd = p.intakeForwardCm * 0.5;
        for (Track t : tracks.values()) {
            double d = Math.hypot(t.x - m[0], t.y - m[1]);
            if (d < bd) { bd = d; best = t; }
        }
        if (best != null) {
            tracks.remove(best.id);
            droppedCaptured++;
        }
    }

    /** Tracks the planner may drive at: confirmed, or close enough to trust. */
    public List<Track> usable(double[] pose) {
        List<Track> out = new ArrayList<>();
        for (Track t : tracks.values()) {
            boolean ok = t.hits >= p.confirmHits;
            if (!ok && pose != null) ok = Math.hypot(t.x - pose[0], t.y - pose[1]) <= p.confirmCloseCm;
            if (ok) out.add(t);
        }
        return out;
    }
}
