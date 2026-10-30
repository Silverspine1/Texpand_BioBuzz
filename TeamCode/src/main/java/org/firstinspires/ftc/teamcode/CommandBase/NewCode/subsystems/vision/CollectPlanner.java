package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Collection planner: which balls, in what order, from where - and the path.
 *
 * The intake is a wide funnel, not a point. A ball is pulled in once it is intakeForwardCm ahead of the
 * chassis centre and any ball within +-intakeSideTolCm of the intake centreline is taken, so the planner
 * plans CAPTURE POSES (where the chassis must be), not ball positions:
 *
 *  - a leg c0 -> c1 takes every free ball whose along-track distance lies in [8, |c1-c0| + forward + 3] and
 *    whose lateral offset from the intake centreline is within the side tolerance. Two balls close together
 *    are therefore one leg, and a ball slightly to one side is taken with the SIDE of the intake instead of
 *    turning to centre it;
 *  - each anchor ball is tried centred and at +-sideTol * lateralOptions; the search keeps whichever offset
 *    shortens the whole route (less turning, shorter path), with a small centerCost so it prefers the middle
 *    when that costs nothing;
 *  - pair mid-points are candidate aims, so a pair is taken in one pass.
 *
 * The best nCandidates plans are then turned into the real curved path and checked against it WITH the heading
 * lag (the intake faces the wrong way while the chassis is still turning): the plan whose real path takes the
 * most balls soonest wins. Heading target = the chord of the leg being driven, switching once its waypoint is
 * passed, so the intake faces the ball at the capture pose.
 *
 * Objective (seconds): T_route + centerCost*sum|lateral| - ballValue * balls_taken. It does NOT consider where the
 * route ends (endValue hook is null / weight 0): it just collects as fast as possible.
 */
public final class CollectPlanner {

    public static final class Plan {
        public final List<double[]> anchors = new ArrayList<>();   // {trackId, x, y}
        public final List<Integer> collects = new ArrayList<>();   // track ids the real path takes, in order
        public final List<Double> laterals = new ArrayList<>();
        public double[][] waypoints;                                // {x, y}, first = start pose
        public double[][] bezier;                                   // per section {p0x,p0y,c1x,c1y,c2x,c2y,p3x,p3y}
        public double[][] polyline;                                 // resampled path, {x, y}
        public double[] polyHeading;                                // heading target per polyline point, deg
        public double tEst, cost;
        public int nExpected;

        public double[] end() { return waypoints[waypoints.length - 1]; }
    }

    /** Optional: value (0..1) of finishing at a field position. null = ignore where the route ends. */
    public interface EndValue { double value(double xCm, double yCm); }

    private final VisionParams p;
    public EndValue endValue = null;
    public double endWeightS = 0.0;            // seconds a certain 1.0 is worth; 0 = ignore the end (teleop default)

    public CollectPlanner(VisionParams p) {
        this.p = p;
    }

    /** Intake centreline offset in the planner's math frame (left of travel = (-uy, ux)); follows the frame handedness. */
    /**
     * The planner works in the DIRECTION OF TRAVEL: the intake always leads. A back intake (intakeForwardCm < 0) is a
     * front intake for a robot reversing into the balls - its distance is |intakeForwardCm|, its side offset is mirrored
     * and the robot heading is the travel heading + BACK_INTAKE_HEADING_OFFSET. Using the signed value put every capture
     * pose 38 cm PAST its ball and bent the path start toward the robot's front - the wrong way for a robot reversing.
     */
    private boolean backIntake() {
        return p.intakeForwardCm < 0;
    }

    private double reach() {
        return Math.abs(p.intakeForwardCm);
    }

    /** Travel heading for a robot heading (deg). */
    public double travelHeading(double robotHeadingDeg) {
        return backIntake() ? robotHeadingDeg + p.backHeadingOffsetDeg : robotHeadingDeg;
    }

    private double off() {
        return (backIntake() ? -1 : 1) * p.intakeOffsetCm * p.lateralSign;
    }

    // ------------------------------------------------------------------------------------------
    // geometry helpers
    // ------------------------------------------------------------------------------------------
    private static double[] unit(double dx, double dy) {
        double n = Math.hypot(dx, dy);
        return n > 1e-9 ? new double[]{dx / n, dy / n} : new double[]{1, 0};
    }

    private double[] clamp(double[] c) {
        double mx = p.wallMarginCm - 2.5;
        return new double[]{Math.max(-p.fieldHalfXCm + mx, Math.min(p.fieldHalfXCm - mx, c[0])),
                Math.max(-p.fieldHalfYCm + mx, Math.min(p.fieldHalfYCm - mx, c[1]))};
    }

    private static double wrap(double a) {
        return ((a + 540.0) % 360.0) - 180.0;
    }

    /** Chassis centre c1 such that the anchor sits intakeForward ahead and {@code lat} to the side of the intake centreline. */
    private double[] capturePose(double[] c0, double[] anchor, double lat) {
        double[] u = unit(anchor[0] - c0[0], anchor[1] - c0[1]);
        double[] c1 = {anchor[0], anchor[1]};
        for (int i = 0; i < 3; i++) {
            double nx = -u[1], ny = u[0];
            // left of the travel direction is (-uy, ux) in the planner (math) frame; lateral offsets are in that frame
            c1 = new double[]{anchor[0] - u[0] * reach() - nx * (lat + off()),
                    anchor[1] - u[1] * reach() - ny * (lat + off())};
            double dx = c1[0] - c0[0], dy = c1[1] - c0[1];
            if (Math.hypot(dx, dy) < 1e-6) break;
            u = unit(dx, dy);
        }
        return c1;
    }

    /** Balls taken by the straight leg c0 -> c1: list of {index, along, lateral}. */
    private List<double[]> swath(double[] c0, double[] c1, double[][] P) {
        double dx = c1[0] - c0[0], dy = c1[1] - c0[1];
        double L = Math.hypot(dx, dy);
        double[] u = L > 1e-6 ? new double[]{dx / L, dy / L} : new double[]{1, 0};
        List<double[]> out = new ArrayList<>();
        for (int i = 0; i < P.length; i++) {
            double rx = P[i][0] - c0[0], ry = P[i][1] - c0[1];
            double along = rx * u[0] + ry * u[1];
            double lat = -rx * u[1] + ry * u[0] - off();
            if (along > 8.0 && along < L + reach() + 3.0 && Math.abs(lat) <= p.intakeSideTolCm)
                out.add(new double[]{i, along, lat});
        }
        return out;
    }

    private double[] legTime(double[] p0, double h0, double[] p1) {
        double dx = p1[0] - p0[0], dy = p1[1] - p0[1];
        double L = Math.hypot(dx, dy);
        double hb = L > 1e-6 ? Math.toDegrees(Math.atan2(dy, dx)) : h0;
        double a = p.maxAccelCmS2, vc = p.maxSpeedCmS * p.cruiseFrac;
        double ramp = vc * vc / (2 * a);
        double tTr = L > ramp ? L / vc + vc / (2 * a) : Math.sqrt(2 * Math.max(L, 0) / a);
        double turn = Math.abs(wrap(hb - h0)) / Math.max(p.headingRateDps, 1.0);
        double extra = Math.max(0.0, turn - p.turnHideFrac * tTr);
        return new double[]{tTr + extra, hb};
    }

    // ------------------------------------------------------------------------------------------
    // search
    // ------------------------------------------------------------------------------------------
    private static final class Cand {
        final double[] anchor; final int[] anc; final double lat;
        Cand(double[] a, int[] n, double l) { anchor = a; anc = n; lat = l; }
    }

    private static final class Node {
        double cost, t; double[] pos; double heading; int mask, got;
        List<Cand> aims = new ArrayList<>(); List<Double> lats = new ArrayList<>();
    }

    /** The nBest best distinct plans by the straight-leg cost (not yet checked against the real path). */
    public List<Plan> planCandidates(double[] pose, List<BallMap.Track> tracks, int room, int nBest) {
        List<Plan> out = new ArrayList<>();
        if (room <= 0 || tracks.isEmpty()) return out;
        // nearest-N tracks
        final double[] pos = {pose[0], pose[1]};
        List<BallMap.Track> sorted = new ArrayList<>(tracks);
        Collections.sort(sorted, (a, b) -> Double.compare(Math.hypot(a.x - pos[0], a.y - pos[1]), Math.hypot(b.x - pos[0], b.y - pos[1])));
        if (sorted.size() > p.consider) sorted = sorted.subList(0, p.consider);
        int n = sorted.size();
        double[][] P = new double[n][];
        int[] ids = new int[n];
        for (int i = 0; i < n; i++) { P[i] = new double[]{sorted.get(i).x, sorted.get(i).y}; ids[i] = sorted.get(i).id; }

        List<Cand> cands = new ArrayList<>();
        for (int j = 0; j < n; j++) {
            for (double f : p.lateralOptions) {
                cands.add(new Cand(P[j], new int[]{j}, f * p.intakeSideTolCm));
                if (f != 0.0) cands.add(new Cand(P[j], new int[]{j}, -f * p.intakeSideTolCm));
            }
        }
        for (int i = 0; i < n; i++)
            for (int j = i + 1; j < n; j++)
                if (Math.hypot(P[i][0] - P[j][0], P[i][1] - P[j][1]) <= 2.0 * p.intakeSideTolCm * 0.95)
                    cands.add(new Cand(new double[]{0.5 * (P[i][0] + P[j][0]), 0.5 * (P[i][1] + P[j][1])}, new int[]{i, j}, 0.0));

        Node start = new Node();
        start.pos = pos; start.heading = travelHeading(pose[2]);
        List<Node> frontier = new ArrayList<>();
        frontier.add(start);
        List<Node> pool = new ArrayList<>();
        for (int depth = 0; depth < p.maxDepth; depth++) {
            List<Node> next = new ArrayList<>();
            for (Node nd : frontier) {
                for (Cand c : cands) {
                    boolean all = true;
                    for (int a : c.anc) if ((nd.mask >> a & 1) == 0) all = false;
                    if (all) continue;
                    double[] c1 = clamp(capturePose(nd.pos, c.anchor, c.lat));
                    double[] lt = legTime(nd.pos, nd.heading, c1);
                    List<double[]> sel = swath(nd.pos, c1, P);
                    List<double[]> take = new ArrayList<>();
                    for (double[] s : sel) if ((nd.mask >> (int) s[0] & 1) == 0) take.add(s);
                    Collections.sort(take, (a, b) -> Double.compare(a[1], b[1]));
                    int room2 = Math.max(room - nd.got, 0);
                    if (take.size() > room2) take = take.subList(0, room2);
                    boolean anchorTaken = false;
                    for (double[] s : take) for (int a : c.anc) if ((int) s[0] == a) anchorTaken = true;
                    if (!anchorTaken) continue;
                    int m2 = nd.mask;
                    double pen = 0;
                    for (double[] s : take) { m2 |= 1 << (int) s[0]; pen += Math.abs(s[2]); }
                    Node ch = new Node();
                    ch.cost = nd.cost + lt[0] - p.ballValueS * take.size() + p.centerCostSPerCm * pen;
                    ch.t = nd.t + lt[0]; ch.pos = c1; ch.heading = lt[1]; ch.mask = m2; ch.got = nd.got + take.size();
                    ch.aims = new ArrayList<>(nd.aims); ch.aims.add(c);
                    ch.lats = new ArrayList<>(nd.lats); ch.lats.add(c.lat);
                    if (endValue != null && endWeightS > 0) ch.cost -= endWeightS * endValue.value(c1[0], c1[1]);
                    next.add(ch);
                    pool.add(ch);
                }
            }
            Collections.sort(next, (a, b) -> Double.compare(a.cost, b.cost));
            frontier = next.size() > p.beam ? new ArrayList<>(next.subList(0, p.beam)) : next;
            if (frontier.isEmpty()) break;
        }
        Collections.sort(pool, (a, b) -> Double.compare(a.cost, b.cost));
        Set<String> seen = new HashSet<>();
        for (Node nd : pool) {
            if (nd.got <= 0) continue;
            StringBuilder sig = new StringBuilder();
            for (int k = 0; k < nd.aims.size(); k++)
                sig.append(Math.round(nd.aims.get(k).anchor[0] * 2)).append(',').append(Math.round(nd.aims.get(k).anchor[1] * 2))
                        .append(',').append(Math.round(nd.lats.get(k))).append(';');
            if (!seen.add(sig.toString())) continue;
            out.add(materialise(pose, P, ids, nd));
            if (out.size() >= Math.max(nBest, 1)) break;
        }
        return out;
    }

    /** A ball nearly touching a wall can only be taken head-on: returns {stageX, stageY, c1x, c1y} or null. */
    private double[] wallStage(double[] prev, double[] ball, double[] c1) {
        double dx = p.fieldHalfXCm - Math.abs(ball[0]), dy = p.fieldHalfYCm - Math.abs(ball[1]);
        if (Math.min(dx, dy) > p.wallTouchCm) return null;
        double[] n = dx <= dy ? new double[]{Math.signum(ball[0]), 0} : new double[]{0, Math.signum(ball[1])};
        double ux = c1[0] - prev[0], uy = c1[1] - prev[1];
        double L = Math.hypot(ux, uy);
        if (L > 1e-6 && (ux * n[0] + uy * n[1]) / L > 0.77) return null;
        double[] cNew = clamp(new double[]{ball[0] - n[0] * reach(), ball[1] - n[1] * reach()});
        double[] st = clamp(new double[]{cNew[0] - n[0] * (p.approachCm + 10), cNew[1] - n[1] * (p.approachCm + 10)});
        if (Math.hypot(st[0] - prev[0], st[1] - prev[1]) < 10 || Math.hypot(cNew[0] - st[0], cNew[1] - st[1]) < 10)
            return new double[]{Double.NaN, Double.NaN, cNew[0], cNew[1]};
        return new double[]{st[0], st[1], cNew[0], cNew[1]};
    }

    private Plan materialise(double[] pose, double[][] P, int[] ids, Node nd) {
        Plan pl = new Plan();
        List<double[]> pts = new ArrayList<>();
        pts.add(new double[]{pose[0], pose[1]});
        int claimed = 0;
        for (int k = 0; k < nd.aims.size(); k++) {
            Cand c = nd.aims.get(k);
            double[] prev = pts.get(pts.size() - 1);
            double[] c1 = clamp(capturePose(prev, c.anchor, c.lat));
            double[] ws = wallStage(prev, c.anchor, c1);
            if (ws != null) {
                if (!Double.isNaN(ws[0])) pts.add(new double[]{ws[0], ws[1]});
                c1 = new double[]{ws[2], ws[3]};
            }
            for (double[] s : swath(prev, c1, P)) claimed |= 1 << (int) s[0];
            pts.add(c1);
            pl.anchors.add(new double[]{ids[c.anc[0]], c.anchor[0], c.anchor[1]});
            pl.laterals.add(c.lat);
        }
        // run a little past the last capture pose so the grip finishes, then stop
        if (pts.size() >= 2) {
            double[] a = pts.get(pts.size() - 2), b = pts.get(pts.size() - 1);
            double L = Math.hypot(b[0] - a[0], b[1] - a[1]);
            if (L > 1e-6 && p.overrunCm > 0)
                pts.add(clamp(new double[]{b[0] + (b[0] - a[0]) / L * p.overrunCm, b[1] + (b[1] - a[1]) / L * p.overrunCm}));
        }
        pl.waypoints = pts.toArray(new double[0][]);
        pl.tEst = nd.t;
        pl.cost = nd.cost;
        pl.nExpected = nd.got;
        return pl;
    }

    // ------------------------------------------------------------------------------------------
    // path (cubic spline through the capture poses) + validation against it
    // ------------------------------------------------------------------------------------------
    /** Fills plan.bezier / polyline / polyHeading for the path starting at pose with the robot heading startHeadingDeg. */
    public void buildPath(Plan pl, double startHeadingDeg) {
        double[][] W = pl.waypoints;
        int n = W.length;
        double[][] tan = new double[n][];
        for (int i = 0; i < n; i++) {
            double tx, ty;
            if (i == 0) {
                double[] b = unit(W[1][0] - W[0][0], W[1][1] - W[0][1]);
                double h = Math.toRadians(startHeadingDeg);
                tx = 0.35 * Math.cos(h) + 0.65 * b[0];
                ty = 0.35 * Math.sin(h) + 0.65 * b[1];
            } else if (i == n - 1) {
                double[] d = unit(W[i][0] - W[i - 1][0], W[i][1] - W[i - 1][1]);
                tx = d[0]; ty = d[1];
            } else {
                double[] a = unit(W[i][0] - W[i - 1][0], W[i][1] - W[i - 1][1]);
                double[] b = unit(W[i + 1][0] - W[i][0], W[i + 1][1] - W[i][1]);
                tx = a[0] + b[0]; ty = a[1] + b[1];
            }
            tan[i] = unit(tx, ty);
        }
        pl.bezier = new double[n - 1][8];
        List<double[]> dense = new ArrayList<>();
        for (int i = 0; i < n - 1; i++) {
            double[] p0 = W[i], p3 = W[i + 1];
            double L = Math.hypot(p3[0] - p0[0], p3[1] - p0[1]);
            double s = p.tangentScale * L;
            double e = Math.min(s, Math.max(p.approachCm, 0.3 * L));
            double[] p1 = {p0[0] + tan[i][0] * s, p0[1] + tan[i][1] * s};
            double[] p2 = {p3[0] - tan[i + 1][0] * e, p3[1] - tan[i + 1][1] * e};
            pl.bezier[i] = new double[]{p0[0], p0[1], p1[0], p1[1], p2[0], p2[1], p3[0], p3[1]};
            int steps = Math.max(12, (int) (L / 1.0));
            for (int k = (i == 0 ? 0 : 1); k <= steps; k++) {
                double u = (double) k / steps, v = 1 - u;
                dense.add(new double[]{v * v * v * p0[0] + 3 * v * v * u * p1[0] + 3 * v * u * u * p2[0] + u * u * u * p3[0],
                        v * v * v * p0[1] + 3 * v * v * u * p1[1] + 3 * v * u * u * p2[1] + u * u * u * p3[1]});
            }
        }
        // resample at ~1 cm
        int m = dense.size();
        double[] cum = new double[m];
        for (int i = 1; i < m; i++) cum[i] = cum[i - 1] + Math.hypot(dense.get(i)[0] - dense.get(i - 1)[0], dense.get(i)[1] - dense.get(i - 1)[1]);
        int cnt = Math.max(2, (int) Math.ceil(cum[m - 1] / 1.0) + 1);
        double[][] poly = new double[cnt][2];
        int j = 0;
        for (int i = 0; i < cnt; i++) {
            double s = Math.min(i * 1.0, cum[m - 1]);
            while (j < m - 2 && cum[j + 1] < s) j++;
            double f = (s - cum[j]) / Math.max(cum[j + 1] - cum[j], 1e-9);
            poly[i][0] = dense.get(j)[0] + f * (dense.get(j + 1)[0] - dense.get(j)[0]);
            poly[i][1] = dense.get(j)[1] + f * (dense.get(j + 1)[1] - dense.get(j)[1]);
        }
        pl.polyline = poly;
        // heading target: the path's own direction headingLookaheadCm further along (what the follower does with
        // path headings) - smooth, and the intake faces along the path at each capture
        if (p.headingLookaheadCm > 0) {
            double[] hl = new double[cnt];
            int look = (int) Math.round(p.headingLookaheadCm);   // polyline is ~1 cm spaced
            for (int i = 0; i < cnt; i++) {
                int a = Math.min(i + look, cnt - 1), b = Math.max(a - 2, 0);
                if (a == b) b = Math.max(a - 1, 0);
                hl[i] = Math.toDegrees(Math.atan2(poly[a][1] - poly[b][1], poly[a][0] - poly[b][0]));
            }
            pl.polyHeading = hl;
            return;
        }
        // (headingLookaheadCm = 0) chord bearing of the leg being driven
        int[] idx = new int[n];
        for (int k = 1; k < n; k++) {
            int best = idx[k - 1];
            double bd = Double.MAX_VALUE;
            for (int i = idx[k - 1]; i < cnt; i++) {
                double d = Math.hypot(poly[i][0] - W[k][0], poly[i][1] - W[k][1]);
                if (d < bd) { bd = d; best = i; }
            }
            idx[k] = best;
        }
        double[] hd = new double[cnt];
        for (int k = 1; k < n; k++) {
            double b = Math.toDegrees(Math.atan2(W[k][1] - W[k - 1][1], W[k][0] - W[k - 1][0]));
            int lo = idx[k - 1], hi = k < n - 1 ? idx[k] + 1 : cnt;
            for (int i = lo; i < hi && i < cnt; i++) hd[i] = b;
        }
        for (int i = 0; i <= idx[0] && i < cnt; i++) hd[i] = cnt > idx[0] + 1 ? hd[idx[0] + 1] : hd[0];
        pl.polyHeading = hd;
    }

    /** Heading target for the robot at (x, y): chord bearing of the leg it is on. */
    public static double headingTarget(Plan pl, double x, double y, int[] legInOut) {
        double[][] W = pl.waypoints;
        int k = Math.max(legInOut[0], 1);
        while (k < W.length - 1) {
            double cx = W[k][0] - W[k - 1][0], cy = W[k][1] - W[k - 1][1];
            double L = Math.hypot(cx, cy);
            double along = L > 1e-6 ? ((x - W[k - 1][0]) * cx + (y - W[k - 1][1]) * cy) / L : L;
            if (along >= L - 6.0) k++; else break;
        }
        legInOut[0] = k;
        return Math.toDegrees(Math.atan2(W[k][1] - W[k - 1][1], W[k][0] - W[k - 1][0]));
    }

    /**
     * Balls the real path takes (with heading lag and speed ramp). Returns list of {time, pathIndex, ballIndex};
     * the path time is returned in totalOut[0].
     */
    public List<double[]> predictCaptures(Plan pl, double h0Deg, double v0, double[][] balls, double[] totalOut) {
        double[][] pts = pl.polyline;
        int n = pts.length;
        double vmax = p.maxSpeedCmS * p.cruiseFrac, acc = p.maxAccelCmS2;
        double wmax = Math.toRadians(p.headingRateDps * p.headingEff);
        double h = Math.toRadians(h0Deg);
        double total = 0;
        for (int i = 1; i < n; i++) total += Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]);
        double t = 0, sCum = 0;
        List<double[]> out = new ArrayList<>();
        boolean[] taken = new boolean[balls.length];
        double v = Math.max(v0, 12.0);
        for (int i = 0; i < n; i++) {
            if (i > 0) {
                double ds = Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]);
                sCum += ds;
                double vRamp = Math.sqrt(v0 * v0 + 2 * acc * sCum);
                double vEnd = Math.sqrt(2 * acc * Math.max(total - sCum, 0)) + 20.0;
                v = Math.max(Math.min(vmax, Math.min(vRamp, vEnd)), 20.0);
                double dt = ds / v;
                t += dt;
                double tgt = Math.toRadians(pl.polyHeading[i]);
                double dh = ((tgt - h + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;
                h += Math.max(-wmax * dt, Math.min(wmax * dt, dh));
            }
            if (i % 2 == 1) continue;
            double tx = Math.cos(h), ty = Math.sin(h);
            double nx = -ty, ny = tx;                                   // planner-frame left of the heading
            double gx = pts[i][0] + tx * reach() + nx * off();
            double gy = pts[i][1] + ty * reach() + ny * off();
            double alongTol = 6.35 + 0.02 * v;
            for (int b = 0; b < balls.length; b++) {
                if (taken[b]) continue;
                double rx = balls[b][0] - gx, ry = balls[b][1] - gy;
                if (Math.abs(rx * tx + ry * ty) < alongTol && Math.abs(rx * nx + ry * ny) <= p.intakeSideTolCm) {
                    taken[b] = true;
                    out.add(new double[]{t, i, b});
                }
            }
        }
        totalOut[0] = t;
        return out;
    }

    /**
     * Build the path for each candidate, check it against the real curved path, and return the best
     * (score = time of the last ball taken - ballValue * balls taken). Fills collects/bezier/polyline. null if none.
     */
    public Plan chooseBest(List<Plan> cands, double[] pose, double speedCmS, List<BallMap.Track> tracks, int room) {
        double[][] B = new double[tracks.size()][];
        for (int i = 0; i < B.length; i++) B[i] = new double[]{tracks.get(i).x, tracks.get(i).y};
        Plan best = null;
        double bestScore = Double.MAX_VALUE;
        for (Plan pl : cands) {
            buildPath(pl, travelHeading(pose[2]));
            double[] tot = new double[1];
            List<double[]> caps = predictCaptures(pl, travelHeading(pose[2]), speedCmS, B, tot);
            if (caps.size() > room) caps = caps.subList(0, room);
            double tCap = caps.isEmpty() ? tot[0] : caps.get(caps.size() - 1)[0];
            double score = tCap - p.ballValueS * caps.size();
            pl.collects.clear();
            for (double[] c : caps) pl.collects.add(tracks.get((int) c[2]).id);
            pl.nExpected = caps.size();
            pl.tEst = tCap;
            if (score < bestScore) { bestScore = score; best = pl; }
        }
        return best;
    }
}
