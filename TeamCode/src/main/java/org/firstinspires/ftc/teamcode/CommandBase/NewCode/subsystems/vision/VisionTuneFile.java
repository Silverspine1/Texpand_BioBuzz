package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.vision;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Properties;

/**
 * Saves / loads every public static double / int / boolean tunable of {@link VisionSubsystem} to
 * {@link VisionSubsystem#TUNE_FILE}. New tunables are picked up automatically. The Vision Tuner OpMode saves
 * (gamepad B); VisionSubsystem.init() loads while USE_TUNED_FILE is true. Values you hard-code as defaults in
 * VisionSubsystem are overridden by a saved file - delete the file (or set USE_TUNED_FILE false) to go back.
 */
public final class VisionTuneFile {

    private VisionTuneFile() { }

    public static void save() throws IOException {
        Properties props = new Properties();
        for (Field f : VisionSubsystem.class.getFields()) {
            if (!isTunable(f)) continue;
            try {
                props.setProperty(f.getName(), String.valueOf(f.get(null)));
            } catch (IllegalAccessException ignored) { }
        }
        File file = new File(VisionSubsystem.TUNE_FILE);
        File dir = file.getParentFile();
        if (dir != null && !dir.exists()) dir.mkdirs();
        try (OutputStream out = new FileOutputStream(file)) {
            props.store(out, "Vision tuning saved from the Vision Tuner");
        }
    }

    /** @return number of values loaded (0 if there is no file). */
    public static int load() {
        File file = new File(VisionSubsystem.TUNE_FILE);
        if (!file.exists()) return 0;
        Properties props = new Properties();
        try (InputStream in = new FileInputStream(file)) {
            props.load(in);
        } catch (IOException e) {
            return 0;
        }
        int n = 0;
        for (Field f : VisionSubsystem.class.getFields()) {
            if (!isTunable(f)) continue;
            String v = props.getProperty(f.getName());
            if (v == null) continue;
            try {
                if (f.getType() == double.class) f.setDouble(null, Double.parseDouble(v));
                else if (f.getType() == int.class) f.setInt(null, Integer.parseInt(v));
                else if (f.getType() == boolean.class) f.setBoolean(null, Boolean.parseBoolean(v));
                else continue;
                n++;
            } catch (Exception ignored) { }
        }
        return n;
    }

    public static boolean delete() {
        return new File(VisionSubsystem.TUNE_FILE).delete();
    }

    private static boolean isTunable(Field f) {
        int m = f.getModifiers();
        if (!Modifier.isStatic(m) || !Modifier.isPublic(m) || Modifier.isFinal(m)) return false;
        Class<?> t = f.getType();
        // USE_TUNED_FILE decides whether the file is read at all, so it is never stored in it;
        // VIEW is a display choice, not a tune
        return (t == double.class || t == int.class || t == boolean.class)
                && !f.getName().equals("USE_TUNED_FILE") && !f.getName().equals("VIEW");
    }
}
