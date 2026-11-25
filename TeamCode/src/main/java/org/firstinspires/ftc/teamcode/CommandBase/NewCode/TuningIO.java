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
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
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

    /** The CURRENT value of every tunable field, as text. Taken once when a tuning class loads = the defaults written in the code. */
    static Map<String, String> snapshot(Class<?> cls, boolean strings) {
        Map<String, String> m = new LinkedHashMap<>();
        for (Field f : cls.getDeclaredFields()) {
            if (!tunable(f, strings)) continue;
            try {
                m.put(f.getName(), String.valueOf(f.get(null)));
            } catch (IllegalAccessException ignored) {
            }
        }
        return m;
    }

    static int save(Class<?> cls, File file, String header, boolean strings, String version) throws IOException {
        return save(cls, file, header, strings, version, null);
    }

    /**
     * Writes every tunable field of cls to file. Written to a temp file (fsync'd), read back and compared, and only then moved over the
     * real file, so a failed save never leaves a half-written file. {@code codeDefaults} (from {@link #snapshot} at class load) is stored
     * as {@code default.<name>}: a later load applies a saved value only while the code default is still that one. Returns the number
     * written; throws if anything fails.
     */
    static int save(Class<?> cls, File file, String header, boolean strings, String version, Map<String, String> codeDefaults) throws IOException {
        Properties p = new Properties();
        if (version != null) p.setProperty("settingsVersion", version);
        int n = 0;
        for (Field f : cls.getDeclaredFields()) {
            if (!tunable(f, strings)) continue;
            try {
                p.setProperty(f.getName(), String.valueOf(f.get(null)));
                if (codeDefaults != null && codeDefaults.containsKey(f.getName())) {
                    p.setProperty(DEFAULT_PREFIX + f.getName(), codeDefaults.get(f.getName()));
                }
                n++;
            } catch (IllegalAccessException ignored) {
            }
        }
        File parent = file.getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs() && !parent.exists()) {
            throw new IOException("could not create " + parent);
        }
        File tmp = new File(file.getPath() + ".tmp");
        try (FileOutputStream out = new FileOutputStream(tmp)) {
            p.store(out, header);
            out.flush();
            out.getFD().sync();
        }
        Properties back = read(tmp);
        List<String> bad = new ArrayList<>();
        for (Field f : cls.getDeclaredFields()) {
            if (!tunable(f, strings)) continue;
            try {
                if (!String.valueOf(f.get(null)).equals(back.getProperty(f.getName()))) bad.add(f.getName());
            } catch (IllegalAccessException ignored) {
            }
        }
        if (!bad.isEmpty()) {
            tmp.delete();
            throw new IOException("written but the read-back differs: " + bad);
        }
        if (!tmp.renameTo(file)) {                       // some file systems will not rename over an existing file
            if (file.exists() && !file.delete()) throw new IOException("could not replace " + file);
            if (!tmp.renameTo(file)) throw new IOException("could not move " + tmp + " to " + file);
        }
        if (!read(file).equals(back)) throw new IOException("the final file does not match what was written: " + file);
        return n;
    }

    static final String DEFAULT_PREFIX = "default.";

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
        /** Saved values NOT applied because the default written in the code has changed since they were saved: the code wins. */
        final List<String> codeWins = new ArrayList<>();
        /** The file has no default.* entries (written by an older version): it cannot be told apart from stale values, so it is ignored. */
        boolean legacy;
    }

    /** Applies the saved values of file over the current ones, except the names in skip and (when upgraded) in resetNames. */
    static Loaded load(Class<?> cls, Properties p, boolean strings, String[] skip, String[] resetNames, boolean upgraded) {
        return load(cls, p, strings, skip, resetNames, upgraded, null);
    }

    /**
     * As above, but with {@code codeDefaults} (from {@link #snapshot} at class load) a saved value is applied only while the code's
     * default for that field is the one that was in the code when it was saved. Change a number in the code and the code value is
     * used until you save again.
     */
    static Loaded load(Class<?> cls, Properties p, boolean strings, String[] skip, String[] resetNames, boolean upgraded,
                       Map<String, String> codeDefaults) {
        Loaded r = new Loaded();
        List<String> skipList = Arrays.asList(skip);
        List<String> resetList = Arrays.asList(resetNames);
        if (codeDefaults != null) {
            boolean any = false;
            for (String k : p.stringPropertyNames()) if (k.startsWith(DEFAULT_PREFIX)) { any = true; break; }
            if (!any) {
                r.legacy = true;
                return r;
            }
        }
        for (Field f : cls.getDeclaredFields()) {
            if (!tunable(f, strings) || skipList.contains(f.getName())) continue;
            if (upgraded && resetList.contains(f.getName())) continue;
            String text = p.getProperty(f.getName());
            if (text == null) continue;
            if (codeDefaults != null) {
                String savedDefault = p.getProperty(DEFAULT_PREFIX + f.getName());
                if (savedDefault == null || !savedDefault.equals(codeDefaults.get(f.getName()))) {
                    r.codeWins.add(f.getName());
                    continue;
                }
            }
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
        if (r.legacy) {
            return what + ": " + file.getPath() + " is an OLD-FORMAT file -> IGNORED, the values in the code are used (save once to write a new file)";
        }
        return what + ": loaded " + r.applied + " saved values from " + file.getPath()
                + (r.codeWins.isEmpty() ? "" : "; " + r.codeWins.size() + " NOT applied because the code default changed since the save (code wins): " + r.codeWins)
                + (r.bad.isEmpty() ? "" : "  !!! UNREADABLE (defaults kept): " + r.bad);
    }
}
