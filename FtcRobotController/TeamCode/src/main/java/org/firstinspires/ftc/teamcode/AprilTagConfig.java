package org.firstinspires.ftc.teamcode;

import org.json.JSONException;
import org.json.JSONObject;

/**
 * Runtime tunables for the AprilTag webcam dashboard.
 *
 * Every field can be changed from the browser while the OpMode is running. Fields that
 * affect the camera or the processor are marked dirty and applied by the OpMode loop
 * (camera work must not happen on the dashboard's HTTP threads).
 */
public class AprilTagConfig {

    public static final String[] RES_NAMES = {"640 x 480", "1280 x 720"};
    public static final int[] RES_W = {640, 1280};
    public static final int[] RES_H = {480, 720};

    public static final long EXPOSURE_AUTO = -1;
    public static final int GAIN_UNCHANGED = -1;
    public static final float DECIMATION_MIN = 1f;
    public static final float DECIMATION_MAX = 4f;
    /** Default pose solver: OPENCV_ITERATIVE is the stable, standard FTC pose solver. */
    public static final String POSE_SOLVER_DEFAULT = "OPENCV_ITERATIVE";

    /** Vision source: a USB webcam through VisionPortal (SDK detection), or a Limelight 3A
     *  polled straight from the HardwareMap (the module does its own detection). */
    public static final String MODE_WEBCAM = "WEBCAM";
    public static final String MODE_LIMELIGHT = "LIMELIGHT";
    public static final String[] MODES = {MODE_WEBCAM, MODE_LIMELIGHT};

    public volatile String cameraName = "Webcam 1";
    /** Which source feeds `latestDetections`. Both reach the same targeting rules. */
    public volatile String cameraMode = MODE_WEBCAM;
    /** HardwareMap device name of the Limelight (FTC Driver Station configuration). */
    public volatile String limelightName = "limelight";
    /** How often the OpMode asks the Limelight for a result, 1..120 Hz. */
    public volatile int limelightPollHz = 20;
    /** Hostname or IP to reach the Limelight with when the device does not report one that
     *  works - a renamed camera is no longer at limelight.local. Empty means "ask the device",
     *  which is what the SDK itself does. */
    public volatile String limelightHost = "";
    /** JSON object of tag id -> name, e.g. {"1":"R_HIVE"}. Limelight results carry no names,
     *  so this is what gives them (and so the alliance rule) something to read. */
    public volatile String tagNames = "{}";
    /** One-shot request: push {@link #tagSizeIn} onto the Limelight's active pipeline. Set by a
     *  dashboard POST, consumed by the OpMode loop, never serialized back - it is an event,
     *  not a setting. */
    public volatile boolean pushTagSize = false;
    public volatile int resolution = 0;
    /** 1 = full resolution. Decimating costs recall: a 24 px tag becomes 12 px at 2x and
     *  drops off the detection list, so full-res is the default. */
    public volatile float decimation = 1f;
    public volatile long exposureUs = EXPOSURE_AUTO;
    public volatile int gain = GAIN_UNCHANGED;
    /** Real-world tag side length in inches - drives every range number. */
    public volatile double tagSizeIn = 5.0;
    /** Horizontal camera FOV in degrees - converts pixels to angles. */
    public volatile double fovDeg = 65.0;
    public volatile String alliance = "RED";
    public volatile double maxRollDeg = 90.0;
    public volatile int minClusterPercent = 0;
    public volatile float minDecisionMargin = 0f;
    public volatile String poseSolver = POSE_SOLVER_DEFAULT;
    public volatile boolean showUnscorable = true;
    public volatile boolean liveView = false;

    /** Pitch (deg) of the hive cell's tag cluster while it is upright - calibrate this. */
    public volatile double hiveBaselinePitchDeg = 0.0;
    /** How far pitch must move from the baseline before the cell counts as tipped. */
    public volatile double hiveTippedThresholdDeg = 25.0;

    private boolean dirty = true;
    /** Parsed copy of {@link #tagNames}; rebuilt whenever the raw string changes. */
    private volatile java.util.Map<Integer, String> tagNameCache = null;
    private String lastTagNames = null;

    /** Merge a partial JSON object (only the keys the dashboard changed) into the config. */
    public synchronized void applyJson(String body) {
        if (body == null || body.isEmpty()) {
            return;
        }
        try {
            JSONObject o = new JSONObject(body);
            if (o.has("cameraName")) cameraName = o.getString("cameraName").trim();
            if (o.has("cameraMode")) cameraMode = normalizeMode(o.getString("cameraMode"));
            if (o.has("limelightName")) limelightName = o.getString("limelightName").trim();
            if (o.has("limelightPollHz")) {
                limelightPollHz = clampInt(o.getInt("limelightPollHz"), 1, 120);
            }
            if (o.has("limelightHost")) {
                limelightHost = sanitizeHost(o.getString("limelightHost"));
            }
            if (o.has("tagNames")) {
                // The dashboard may send the map as an object or as a pre-encoded string;
                // either way only a well-formed object is ever stored, so toJson stays valid.
                Object t = o.get("tagNames");
                String s = (t instanceof JSONObject) ? t.toString() : String.valueOf(t);
                if (isJsonObject(s)) {
                    tagNames = s;
                    tagNameCache = null;
                }
            }
            if (o.has("pushTagSize") && o.getBoolean("pushTagSize")) {
                pushTagSize = true;
            }
            if (o.has("resolution")) resolution = clampInt(o.getInt("resolution"), 0, RES_W.length - 1);
            if (o.has("decimation")) decimation = (float) o.getDouble("decimation");
            if (o.has("tagSizeIn")) tagSizeIn = clamp(o.getDouble("tagSizeIn"), 0.1, 48.0);
            if (o.has("fovDeg")) fovDeg = clamp(o.getDouble("fovDeg"), 20.0, 170.0);
            if (o.has("exposureUs")) exposureUs = o.getLong("exposureUs");
            if (o.has("gain")) gain = o.getInt("gain");
            if (o.has("alliance")) alliance = o.getString("alliance").toUpperCase();
            if (o.has("maxRollDeg")) maxRollDeg = o.getDouble("maxRollDeg");
            if (o.has("minClusterPercent")) minClusterPercent = clampInt(o.getInt("minClusterPercent"), 0, 100);
            if (o.has("minDecisionMargin")) minDecisionMargin = (float) o.getDouble("minDecisionMargin");
            if (o.has("poseSolver")) poseSolver = o.getString("poseSolver");
            if (o.has("showUnscorable")) showUnscorable = o.getBoolean("showUnscorable");
            if (o.has("liveView")) liveView = o.getBoolean("liveView");
            if (o.has("hiveBaselinePitchDeg")) hiveBaselinePitchDeg = o.getDouble("hiveBaselinePitchDeg");
            if (o.has("hiveTippedThresholdDeg")) {
                hiveTippedThresholdDeg = Math.max(1.0, o.getDouble("hiveTippedThresholdDeg"));
            }
            dirty = true;
        } catch (JSONException ignored) {
            // A malformed dashboard POST must never kill the OpMode.
        }
    }

    /** Returns true once after any change, so the OpMode applies camera work exactly once. */
    public synchronized boolean consumeDirty() {
        boolean d = dirty;
        dirty = false;
        return d;
    }

    /** Returns true once per dashboard request, so the Limelight round-trip runs exactly once. */
    public synchronized boolean consumePushTagSize() {
        boolean b = pushTagSize;
        pushTagSize = false;
        return b;
    }

    public void resetToDefaults() {
        AprilTagConfig d = new AprilTagConfig();
        cameraName = d.cameraName;
        cameraMode = d.cameraMode;
        limelightName = d.limelightName;
        limelightPollHz = d.limelightPollHz;
        limelightHost = d.limelightHost;
        tagNames = d.tagNames;
        tagNameCache = null;
        pushTagSize = false;
        resolution = d.resolution;
        decimation = d.decimation;
        exposureUs = d.exposureUs;
        gain = d.gain;
        tagSizeIn = d.tagSizeIn;
        fovDeg = d.fovDeg;
        alliance = d.alliance;
        maxRollDeg = d.maxRollDeg;
        minClusterPercent = d.minClusterPercent;
        minDecisionMargin = d.minDecisionMargin;
        poseSolver = d.poseSolver;
        showUnscorable = d.showUnscorable;
        liveView = d.liveView;
        hiveBaselinePitchDeg = d.hiveBaselinePitchDeg;
        hiveTippedThresholdDeg = d.hiveTippedThresholdDeg;
        dirty = true;
    }

    public String toJson() {
        StringBuilder sb = new StringBuilder(384);
        sb.append('{');
        field(sb, "cameraName", cameraName).append(',');
        field(sb, "cameraMode", cameraMode).append(',');
        field(sb, "limelightName", limelightName).append(',');
        field(sb, "limelightPollHz", (double) limelightPollHz).append(',');
        field(sb, "limelightHost", limelightHost).append(',');
        sb.append("\"tagNames\":").append(isJsonObject(tagNames) ? tagNames : "{}").append(',');
        field(sb, "resolution", (double) resolution).append(',');
        field(sb, "decimation", decimation, 1).append(',');
        field(sb, "exposureUs", (double) exposureUs).append(',');
        field(sb, "gain", (double) gain).append(',');
        field(sb, "tagSizeIn", tagSizeIn, 2).append(',');
        field(sb, "fovDeg", fovDeg, 1).append(',');
        field(sb, "alliance", alliance).append(',');
        field(sb, "maxRollDeg", maxRollDeg, 1).append(',');
        field(sb, "minClusterPercent", (double) minClusterPercent).append(',');
        field(sb, "minDecisionMargin", minDecisionMargin, 1).append(',');
        field(sb, "poseSolver", poseSolver).append(',');
        field(sb, "showUnscorable", showUnscorable).append(',');
        field(sb, "liveView", liveView).append(',');
        field(sb, "hiveBaselinePitchDeg", hiveBaselinePitchDeg, 1).append(',');
        field(sb, "hiveTippedThresholdDeg", hiveTippedThresholdDeg, 1);
        sb.append('}');
        return sb.toString();
    }

    /**
     * BIOBUZZ hive-cell tip check.
     *
     * The 4-tag AprilTag cluster is mounted on the UNDERSIDE of the cell, so the cluster's
     * pose rides along with the cell: when the cell rocks over, the whole cluster's pitch
     * shifts by (at least) the mechanical tilt angle of the structure.
     *
     * Calibrate `baselinePitchDeg` while the cell is at rest upright, then flag any cell
     * whose pitch has moved more than `thresholdDeg` from that baseline. Returns false when
     * nothing is detected, so an empty frame never reads as "tipped".
     */
    public static boolean isTipped(double pitchDeg, double baselinePitchDeg, double thresholdDeg) {
        if (Double.isNaN(pitchDeg) || Double.isInfinite(pitchDeg)) {
            return false;
        }
        return Math.abs(pitchDeg - baselinePitchDeg) > Math.abs(thresholdDeg);
    }

    /** {@return true when detections come from a Limelight rather than a USB webcam}. */
    public boolean isLimelight() {
        return MODE_LIMELIGHT.equals(cameraMode);
    }

    /** Only WEBCAM and LIMELIGHT exist; anything else in a POST falls back to WEBCAM. */
    static String normalizeMode(String raw) {
        String m = raw == null ? "" : raw.trim().toUpperCase(java.util.Locale.US);
        for (String known : MODES) {
            if (known.equals(m)) {
                return known;
            }
        }
        return MODE_WEBCAM;
    }

    /**
     * A host that can be put straight into a URL: no scheme, no path, no port.
     *
     * <p>People type what the browser shows ("https://limelight.local:5801/"), but the link and
     * the REST call each add their own port, so everything except the host itself is dropped -
     * otherwise the two would disagree about where the camera is.
     */
    static String sanitizeHost(String raw) {
        if (raw == null) {
            return "";
        }
        String s = raw.trim().replaceFirst("(?i)^https?://", "");
        int slash = s.indexOf('/');
        if (slash >= 0) {
            s = s.substring(0, slash);
        }
        if (s.matches(".*:\\d{1,5}$")) {
            s = s.replaceAll(":\\d{1,5}$", "");
        }
        s = s.trim();
        return s.length() > 64 ? s.substring(0, 64) : s;
    }

    static boolean isJsonObject(String s) {
        if (s == null) {
            return false;
        }
        try {
            new JSONObject(s);
            return true;
        } catch (JSONException e) {
            return false;
        }
    }

    /**
     * {@return the id -> name table from {@link #tagNames}}, parsed once per edit.
     *
     * The SDK attaches names to webcam detections itself; Limelight results arrive nameless,
     * so the bridge looks names up here and the alliance rule still has something to read.
     */
    public synchronized java.util.Map<Integer, String> tagNameMap() {
        String raw = tagNames == null ? "{}" : tagNames;
        if (tagNameCache != null && raw.equals(lastTagNames)) {
            return tagNameCache;
        }
        java.util.Map<Integer, String> parsed = new java.util.TreeMap<>();
        try {
            JSONObject o = new JSONObject(raw);
            for (java.util.Iterator<String> it = o.keys(); it.hasNext(); ) {
                String k = it.next();
                try {
                    parsed.put(Integer.valueOf(k.trim()), o.getString(k));
                } catch (NumberFormatException | JSONException ignored) {
                    // a key that is not an id is simply not part of the table
                }
            }
        } catch (JSONException ignored) {
            // never stored malformed, but be safe: no names beats a thrown loop
        }
        tagNameCache = parsed;
        lastTagNames = raw;
        return parsed;
    }

    /** {@return the configured name for this tag id, or null when the table has none}. */
    public String tagName(int id) {
        String n = tagNameMap().get(id);
        return n == null || n.isEmpty() ? null : n;
    }

    static StringBuilder field(StringBuilder sb, String key, String value) {
        sb.append('"').append(key).append("\":");
        if (value == null) {
            sb.append("null");
        } else {
            sb.append('"').append(escape(value)).append('"');
        }
        return sb;
    }

    static StringBuilder field(StringBuilder sb, String key, double value) {
        return field(sb, key, (double) value, 0);
    }

    static StringBuilder field(StringBuilder sb, String key, double value, int decimals) {
        sb.append('"').append(key).append("\":");
        if (Double.isNaN(value) || Double.isInfinite(value)) {
            sb.append("null");
        } else if (decimals <= 0) {
            sb.append((long) Math.round(value));
        } else {
            sb.append(String.format(java.util.Locale.US, "%." + decimals + "f", value));
        }
        return sb;
    }

    static StringBuilder field(StringBuilder sb, String key, boolean value) {
        sb.append('"').append(key).append("\":").append(value);
        return sb;
    }

    static String escape(String s) {
        StringBuilder sb = new StringBuilder(s.length() + 8);
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            switch (c) {
                case '"': sb.append("\\\""); break;
                case '\\': sb.append("\\\\"); break;
                case '\n': sb.append("\\n"); break;
                case '\r': sb.append("\\r"); break;
                case '\t': sb.append("\\t"); break;
                default:
                    if (c < 0x20) {
                        sb.append(String.format(java.util.Locale.US, "\\u%04x", (int) c));
                    } else {
                        sb.append(c);
                    }
            }
        }
        return sb.toString();
    }

    private static int clampInt(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double clamp(double v, double lo, double hi) {
        if (Double.isNaN(v) || Double.isInfinite(v)) {
            return lo;
        }
        return Math.max(lo, Math.min(hi, v));
    }
}
