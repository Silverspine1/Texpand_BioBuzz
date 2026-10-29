package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.Drive.Pathing;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Per-loop CSV log of every Follower.followPathAuto call, plus extra columns
 * (DriveBase registers its wheel commands / braking ramp).
 *
 * OpModeEX calls start() in init and stop() in stop. The file is only created
 * once a follower actually runs, in DIR, named <OpMode>_<date>_<time>.csv.
 * Pull with:  adb pull /sdcard/FIRST/pathing_logs
 */
public final class PathLog {

    public static boolean ENABLED = true;
    public static final String DIR = "/sdcard/FIRST/pathing_logs";

    /** Extra columns appended to every row. */
    public interface Extras {
        String[] names();
        void values(double[] out);
    }

    private static final String[] FOLLOWER_COLUMNS = {
            "t", "run", "dt", "mode", "limiter", "idx",
            "x", "y", "h", "vx", "vy",
            "endX", "endY", "err", "errAlong", "errCross", "toGo",
            "vTarget", "vCmdF", "vCmdS", "aCmdF", "aCmdS", "vMeasF", "vMeasS",
            "pF", "pS", "turn", "hdgTarget", "hdgErr", "hdgRate", "hdgHold"
    };

    private static String tag = null;
    private static BufferedWriter out = null;
    private static Extras extras = null;
    private static double[] extraVals = new double[0];
    private static String note = "";
    private static long t0 = 0, lastFlush = 0;
    private static int run = 0;
    private static boolean failed = false;
    private static final StringBuilder sb = new StringBuilder(512);

    private PathLog() {}

    /** Arms logging for this OpMode; the file opens on the first row. */
    public static synchronized void start(String opModeName) {
        stop();
        tag = opModeName == null ? "log" : opModeName.replaceAll("[^A-Za-z0-9_-]", "");
        failed = false;
        run = 0;
        note = "";
    }

    public static synchronized void stop() {
        if (out != null) {
            try {
                out.flush();
                out.close();
            } catch (IOException ignored) {}
        }
        out = null;
        tag = null;
    }

    public static synchronized void setExtras(Extras e) {
        extras = e;
        extraVals = new double[e == null ? 0 : e.names().length];
    }

    /** Free text written in the "note" column of following rows (commas removed). */
    public static synchronized void note(String n) {
        note = n == null ? "" : n.replace(',', ' ').replace('\n', ' ');
    }

    /** Marks the start of a new path (Follower.setPath). */
    public static synchronized void newRun() {
        run++;
    }

    public static boolean active() {
        return ENABLED && tag != null && !failed;
    }

    /** Called by Follower with values in FOLLOWER_COLUMNS order. */
    public static synchronized void row(double... v) {
        if (!active()) return;
        long now = System.nanoTime();
        if (out == null && !open(now)) return;
        sb.setLength(0);
        sb.append(fmt((now - t0) / 1e9)).append(',').append(run);
        for (int i = 2; i < v.length; i++) sb.append(',').append(fmt(v[i]));
        if (extras != null) {
            try {
                extras.values(extraVals);
            } catch (RuntimeException ignored) {}
            for (double d : extraVals) sb.append(',').append(fmt(d));
        }
        sb.append(',').append(note).append('\n');
        try {
            out.write(sb.toString());
            if (now - lastFlush > 1_000_000_000L) {
                out.flush();
                lastFlush = now;
            }
        } catch (IOException e) {
            failed = true;
        }
    }

    private static boolean open(long now) {
        try {
            File dir = new File(DIR);
            if (!dir.exists()) dir.mkdirs();
            String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            out = new BufferedWriter(new FileWriter(new File(dir, tag + "_" + stamp + ".csv")), 1 << 16);
            StringBuilder h = new StringBuilder();
            for (int i = 0; i < FOLLOWER_COLUMNS.length; i++) {
                if (i > 0) h.append(',');
                h.append(FOLLOWER_COLUMNS[i]);
            }
            if (extras != null) for (String s : extras.names()) h.append(',').append(s);
            h.append(",note\n");
            out.write(h.toString());
            t0 = now;
            lastFlush = now;
            return true;
        } catch (IOException e) {
            failed = true;
            out = null;
            return false;
        }
    }

    public static int columnCount() {
        return FOLLOWER_COLUMNS.length;
    }

    private static String fmt(double v) {
        if (Double.isNaN(v)) return "";
        if (Math.abs(v) < 5e-5) return "0";
        if (Math.abs(v) >= 1e6) return Long.toString(Math.round(v));
        return Double.toString(Math.round(v * 1e4) / 1e4);
    }
}
