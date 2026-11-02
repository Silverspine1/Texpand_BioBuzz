package org.firstinspires.ftc.teamcode.CommandBase.NewCode;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Properties;

/**
 * Save / load of a tuning class (ShotControlTuning, ShooterTuning): every public static non-final double / boolean / String field,
 * one properties file. Pure Java (no SDK) so tools/shot_java_parity/SettingsRoundTrip tests the real code. A save is READ BACK and
 * compared; a load reports what it did, so the displays can say whether the hub really has your numbers.
 */
final class TuningIO {
    private TuningIO() {}

    static boolean tunable(Field f, boolean strings) {
        int m = f.getModifiers();
        return Modifier.isPublic(m) && Modifier.isStatic(m) && !Modifier.isFinal(m)
                && (f.getType() == double.class || f.getType() == boolean.class || (strings && f.getType() == String.class));
    }

    /** Writes every tunable field of cls to file (fsync'd). Returns the number written; throws if the read-back differs. */
    static int save(Class<?> cls, File file, String header, boolean strings, String version) throws IOException {
        Properties p = new Properties();
        if (version != null) p.setProperty("settingsVersion", version);
        int n = 0;
        for (Field f : cls.getDeclaredFields()) {
            if (!tunable(f, strings)) continue;
            try {
                p.setProperty(f.getName(), String.valueOf(f.get(null)));
                n++;
            } catch (IllegalAccessException ignored) {
            }
        }
        File parent = file.getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs()) {
            throw new IOException("could not create " + parent);
        }
        try (FileOutputStream out = new FileOutputStream(file)) {
            p.store(out, header);
            out.flush();
            out.getFD().sync();
        }
        Properties back = read(file);
        List<String> bad = new ArrayList<>();
        for (Field f : cls.getDeclaredFields()) {
            if (!tunable(f, strings)) continue;
            try {
                if (!String.valueOf(f.get(null)).equals(back.getProperty(f.getName()))) bad.add(f.getName());
            } catch (IllegalAccessException ignored) {
            }
        }
        if (!bad.isEmpty()) throw new IOException("written but the read-back differs: " + bad);
        return n;
    }

    static Properties read(File file) throws IOException {
        Properties p = new Properties();
        try (InputStream in = new FileInputStream(file)) {
            p.load(in);
        }
        return p;
    }

    /** Result of a load: how many values were applied and which saved values could not be read. */
    static final class Loaded {
        int applied;
        final List<String> bad = new ArrayList<>();
    }

    /** Applies the saved values of file over the current ones, except the names in skip and (when upgraded) in resetNames. */
    static Loaded load(Class<?> cls, Properties p, boolean strings, String[] skip, String[] resetNames, boolean upgraded) {
        Loaded r = new Loaded();
        List<String> skipList = Arrays.asList(skip);
        List<String> resetList = Arrays.asList(resetNames);
        for (Field f : cls.getDeclaredFields()) {
            if (!tunable(f, strings) || skipList.contains(f.getName())) continue;
            if (upgraded && resetList.contains(f.getName())) continue;
            String text = p.getProperty(f.getName());
            if (text == null) continue;
            try {
                if (f.getType() == boolean.class) {
                    f.setBoolean(null, Boolean.parseBoolean(text));
                } else if (f.getType() == String.class) {
                    f.set(null, text.trim());
                } else {
                    double v = Double.parseDouble(text);
                    if (Double.isNaN(v) || Double.isInfinite(v)) {
                        r.bad.add(f.getName());
                        continue;
                    }
                    f.setDouble(null, v);
                }
                r.applied++;
            } catch (IllegalAccessException | NumberFormatException e) {
                r.bad.add(f.getName());
            }
        }
        return r;
    }

    static String describe(String what, File file, Loaded r) {
        return what + ": loaded " + r.applied + " saved values from " + file.getPath()
                + (r.bad.isEmpty() ? "" : "  !!! UNREADABLE (defaults kept): " + r.bad);
    }
}
