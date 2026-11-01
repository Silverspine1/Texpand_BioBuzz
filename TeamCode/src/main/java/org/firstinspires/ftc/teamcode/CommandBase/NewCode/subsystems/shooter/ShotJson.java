package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Minimal JSON reader for the shot-control model files (no external libraries on the robot).
 * Objects become {@code Map<String, Object>}, arrays {@code List<Object>}, numbers {@code Double},
 * plus String / Boolean / null. Accepts Python's non-standard {@code NaN}, {@code Infinity},
 * {@code -Infinity} (json.dumps writes them). Load-time only: it allocates freely.
 * Numbers go through {@link Double#parseDouble}, so Python's repr floats round-trip exactly.
 */
public final class ShotJson {

    private final String s;
    private int i;

    private ShotJson(String s) {
        this.s = s;
    }

    public static Object parse(String text) {
        ShotJson p = new ShotJson(text);
        p.ws();
        Object v = p.value();
        p.ws();
        if (p.i != p.s.length()) throw p.err("trailing characters");
        return v;
    }

    private RuntimeException err(String what) {
        return new IllegalArgumentException("JSON: " + what + " at offset " + i);
    }

    private void ws() {
        while (i < s.length()) {
            char c = s.charAt(i);
            if (c == ' ' || c == '\n' || c == '\r' || c == '\t') i++;
            else break;
        }
    }

    private Object value() {
        if (i >= s.length()) throw err("unexpected end");
        char c = s.charAt(i);
        switch (c) {
            case '{': return object();
            case '[': return array();
            case '"': return string();
            case 't': expect("true"); return Boolean.TRUE;
            case 'f': expect("false"); return Boolean.FALSE;
            case 'n': expect("null"); return null;
            case 'N': expect("NaN"); return Double.NaN;
            case 'I': expect("Infinity"); return Double.POSITIVE_INFINITY;
            default: return number();
        }
    }

    private void expect(String word) {
        if (!s.startsWith(word, i)) throw err("expected " + word);
        i += word.length();
    }

    private Map<String, Object> object() {
        Map<String, Object> m = new LinkedHashMap<String, Object>();
        i++;
        ws();
        if (i < s.length() && s.charAt(i) == '}') { i++; return m; }
        while (true) {
            ws();
            if (i >= s.length() || s.charAt(i) != '"') throw err("expected key");
            String k = string();
            ws();
            if (i >= s.length() || s.charAt(i) != ':') throw err("expected ':'");
            i++;
            ws();
            m.put(k, value());
            ws();
            if (i >= s.length()) throw err("unterminated object");
            char c = s.charAt(i++);
            if (c == '}') return m;
            if (c != ',') throw err("expected ',' or '}'");
        }
    }

    private List<Object> array() {
        List<Object> a = new ArrayList<Object>();
        i++;
        ws();
        if (i < s.length() && s.charAt(i) == ']') { i++; return a; }
        while (true) {
            ws();
            a.add(value());
            ws();
            if (i >= s.length()) throw err("unterminated array");
            char c = s.charAt(i++);
            if (c == ']') return a;
            if (c != ',') throw err("expected ',' or ']'");
        }
    }

    private String string() {
        i++;
        StringBuilder b = new StringBuilder();
        while (true) {
            if (i >= s.length()) throw err("unterminated string");
            char c = s.charAt(i++);
            if (c == '"') return b.toString();
            if (c != '\\') { b.append(c); continue; }
            char e = s.charAt(i++);
            switch (e) {
                case 'n': b.append('\n'); break;
                case 't': b.append('\t'); break;
                case 'r': b.append('\r'); break;
                case 'b': b.append('\b'); break;
                case 'f': b.append('\f'); break;
                case 'u': b.append((char) Integer.parseInt(s.substring(i, i + 4), 16)); i += 4; break;
                default: b.append(e);
            }
        }
    }

    private Double number() {
        int start = i;
        if (s.charAt(i) == '-') {
            i++;
            if (s.startsWith("Infinity", i)) { i += 8; return Double.NEGATIVE_INFINITY; }
        }
        while (i < s.length()) {
            char c = s.charAt(i);
            if ((c >= '0' && c <= '9') || c == '.' || c == 'e' || c == 'E' || c == '+' || c == '-') i++;
            else break;
        }
        if (start == i) throw err("unexpected character '" + s.charAt(i) + "'");
        return Double.parseDouble(s.substring(start, i));
    }

    // ---- typed access helpers (throw with the key name when the file does not match) -------------------------

    @SuppressWarnings("unchecked")
    public static Map<String, Object> obj(Object o, String key) {
        Object v = ((Map<String, Object>) o).get(key);
        if (!(v instanceof Map)) throw new IllegalArgumentException("missing object '" + key + "'");
        return (Map<String, Object>) v;
    }

    @SuppressWarnings("unchecked")
    public static boolean has(Object o, String key) {
        return ((Map<String, Object>) o).get(key) != null;
    }

    @SuppressWarnings("unchecked")
    public static double num(Object o, String key) {
        Object v = ((Map<String, Object>) o).get(key);
        if (!(v instanceof Double)) throw new IllegalArgumentException("missing number '" + key + "'");
        return (Double) v;
    }

    @SuppressWarnings("unchecked")
    public static double num(Object o, String key, double fallback) {
        Object v = ((Map<String, Object>) o).get(key);
        return v instanceof Double ? (Double) v : fallback;
    }

    @SuppressWarnings("unchecked")
    public static String str(Object o, String key, String fallback) {
        Object v = ((Map<String, Object>) o).get(key);
        return v instanceof String ? (String) v : fallback;
    }

    @SuppressWarnings("unchecked")
    public static List<Object> list(Object o, String key) {
        Object v = ((Map<String, Object>) o).get(key);
        if (!(v instanceof List)) throw new IllegalArgumentException("missing array '" + key + "'");
        return (List<Object>) v;
    }

    /** 1-D numeric array. */
    public static double[] vec(List<Object> a) {
        double[] out = new double[a.size()];
        for (int k = 0; k < out.length; k++) out[k] = toD(a.get(k));
        return out;
    }

    /** 2-D array [n0][n1], row-major flattened; checks the shape. */
    @SuppressWarnings("unchecked")
    public static double[] flat2(List<Object> a, int n0, int n1, String name) {
        if (a.size() != n0) throw new IllegalArgumentException(name + ": expected " + n0 + " rows, got " + a.size());
        double[] out = new double[n0 * n1];
        for (int p = 0; p < n0; p++) {
            List<Object> row = (List<Object>) a.get(p);
            if (row.size() != n1) throw new IllegalArgumentException(name + ": row " + p + " has " + row.size() + " values, expected " + n1);
            for (int q = 0; q < n1; q++) out[p * n1 + q] = toD(row.get(q));
        }
        return out;
    }

    /** 3-D array [n0][n1][n2], row-major flattened; checks the shape. */
    @SuppressWarnings("unchecked")
    public static double[] flat3(List<Object> a, int n0, int n1, int n2, String name) {
        if (a.size() != n0) throw new IllegalArgumentException(name + ": expected " + n0 + " planes, got " + a.size());
        double[] out = new double[n0 * n1 * n2];
        for (int p = 0; p < n0; p++) {
            double[] plane = flat2((List<Object>) a.get(p), n1, n2, name + "[" + p + "]");
            System.arraycopy(plane, 0, out, p * n1 * n2, n1 * n2);
        }
        return out;
    }

    private static double toD(Object v) {
        if (v == null) return Double.NaN;          // a null in a numeric table = missing
        if (v instanceof Double) return (Double) v;
        if (v instanceof Boolean) return ((Boolean) v) ? 1.0 : 0.0;
        throw new IllegalArgumentException("expected a number, got " + v);
    }
}
