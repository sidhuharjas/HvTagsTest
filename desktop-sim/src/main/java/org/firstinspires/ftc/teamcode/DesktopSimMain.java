package org.firstinspires.ftc.teamcode;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.util.Locale;

/**
 * Desktop "robot simulator" for the AprilTag dashboard - runs on the pit laptop, no robot
 * needed. Serves the exact same dashboard page on localhost:8080 that the robot OpMode
 * serves, so you can learn the UI and tune targeting rules before you deploy.
 *
 * Detection sources (checked in this order):
 *   1. Live webcam - run webcam_tags.py (your laptop camera becomes the robot camera and
 *      prints/detects real AprilTags, tag36h11 family); detections arrive on :8081.
 *   2. Synthetic - animated fake BIOBUZZ-style clusters (RED/BLUE, upright and upside-down)
 *      so the dashboard is never empty.
 *
 * Run from Android Studio (desktop-sim module, Run button) or:
 *   powershell -File desktop-sim/run-sim.ps1
 */
public class DesktopSimMain {

    public static final int DASHBOARD_PORT = AprilTagDashboardServer.PORT;
    public static final int INGEST_PORT = 8081;

    private static final AprilTagConfig config = new AprilTagConfig();
    private static volatile String ingestPayload;
    private static volatile long ingestAtMs;
    private static volatile String latestState = "{}";
    private static String lastMode = "synthetic";

    public static void main(String[] args) throws Exception {
        startIngestServer();
        AprilTagDashboardServer server = new AprilTagDashboardServer(new AprilTagDashboardServer.Backend() {
            @Override
            public String snapshotJson() {
                return latestState;
            }

            @Override
            public void applyConfig(String jsonBody) {
                config.applyJson(jsonBody);
            }
        });

        System.out.println("AprilTag desktop simulator");
        System.out.println("  dashboard:  http://localhost:" + server.port());
        System.out.println("  ingest:     http://localhost:" + INGEST_PORT + "/sim/detections");
        System.out.println("  webcam:     python desktop-sim/webcam_tags.py   (laptop camera = robot camera)");
        System.out.println("  stopping:   Ctrl+C");

        long last = System.nanoTime();
        while (true) {
            long nowNanos = System.nanoTime();
            double loopMs = (nowNanos - last) / 1_000_000.0;
            last = nowNanos;
            latestState = buildState(loopMs);
            long sleep = 33 - (System.nanoTime() - nowNanos) / 1_000_000;
            if (sleep > 0) {
                Thread.sleep(sleep);
            }
        }
    }

    /* ---------------- webcam ingest (:8081) ---------------- */

    private static void startIngestServer() throws IOException {
        HttpServer ingest = HttpServer.create(new InetSocketAddress(INGEST_PORT), 0);
        ingest.createContext("/sim/detections", new HttpHandler() {
            @Override
            public void handle(HttpExchange exchange) throws IOException {
                try {
                    if ("POST".equals(exchange.getRequestMethod())) {
                        ingestPayload = readBody(exchange.getRequestBody());
                        ingestAtMs = System.currentTimeMillis();
                        respond(exchange, 200, "{\"ok\":true}");
                    } else {
                        respond(exchange, 200, "{\"ok\":true,\"hint\":\"POST detections here\"}");
                    }
                } catch (Exception e) {
                    respond(exchange, 500, "{\"ok\":false}");
                } finally {
                    exchange.close();
                }
            }
        });
        // The webcam script polls this so dashboard sliders (exposure/gain/decimation/
        // resolution) reach the actual camera instead of being robot-only controls.
        ingest.createContext("/sim/config", new HttpHandler() {
            @Override
            public void handle(HttpExchange exchange) throws IOException {
                try {
                    respond(exchange, 200, config.toJson());
                } finally {
                    exchange.close();
                }
            }
        });
        ingest.setExecutor(java.util.concurrent.Executors.newSingleThreadExecutor());
        ingest.start();
    }

    /* ---------------- state JSON ---------------- */

    private static String buildState(double loopMs) {
        boolean webcamLive = ingestPayload != null && System.currentTimeMillis() - ingestAtMs < 750;
        if (webcamLive) {
            lastMode = "webcam";
        } else if (!"synthetic".equals(lastMode)) {
            lastMode = "synthetic"; // webcam went quiet - fall back so the page is never empty
        }

        StringBuilder sb = new StringBuilder(4096);
        sb.append('{');
        AprilTagConfig.field(sb, "camState", webcamLive ? "STREAMING (webcam)" : "STREAMING (synthetic)").append(',');
        AprilTagConfig.field(sb, "fps", 30.0, 1).append(',');
        AprilTagConfig.field(sb, "loopMs", loopMs, 1).append(',');
        AprilTagConfig.field(sb, "ip", "localhost").append(',');
        // webcam_tags.py serves the annotated preview on its own port; on the robot the
        // OpMode serves /stream.mjpg from its dashboard port instead.
        AprilTagConfig.field(sb, "streamUrl", "http://localhost:8090/stream.mjpg").append(',');
        AprilTagConfig.field(sb, "expMinUs", 5000.0).append(',');
        AprilTagConfig.field(sb, "expMaxUs", 1000000.0).append(',');
        AprilTagConfig.field(sb, "gainMin", 0.0).append(',');
        AprilTagConfig.field(sb, "gainMax", 255.0).append(',');
        sb.append("\"config\":").append(config.toJson()).append(',');

        JSONArray dets = webcamLive ? parseIngest() : syntheticDetections();
        JSONArray shown = new JSONArray();
        JSONObject target = null;
        JSONObject nearest = null;
        double nearestRange = Double.MAX_VALUE;
        double bestRange = Double.MAX_VALUE;

        for (int i = 0; i < dets.length(); i++) {
            JSONObject d = dets.getJSONObject(i);
            boolean scorable = isScorable(d);
            d.put("scorable", scorable);

            // The hive readout tracks the nearest cell by ANY means, scorable or not: a cell
            // that has tipped is precisely the one whose tags go unscorable, so filtering it
            // out here would hide the case we care about.
            double rangeNow = d.optDouble("range", Double.MAX_VALUE);
            if (rangeNow < nearestRange) {
                nearestRange = rangeNow;
                nearest = d;
            }

            if (!scorable && !config.showUnscorable) {
                continue;
            }
            shown.put(d);
            if (scorable && rangeNow < bestRange) {
                bestRange = rangeNow;
                target = d;
            }
        }

        sb.append("\"target\":").append(target == null ? "null" : target.toString()).append(',');
        AprilTagConfig.field(sb, "targetWhy",
                target == null ? noTargetWhy(dets, nearest) : "").append(',');
        // BIOBUZZ hive-cell tip check: the cluster rides the underside of the cell, so its
        // pitch tracks the cell. Compare against the calibrated upright baseline.
        double hivePitch = Double.NaN;
        boolean isTipped = false;
        if (nearest != null) {
            hivePitch = nearest.optDouble("pitch", Double.NaN);
            isTipped = AprilTagConfig.isTipped(hivePitch, config.hiveBaselinePitchDeg,
                    config.hiveTippedThresholdDeg);
        }
        sb.append("\"hive\":{");
        AprilTagConfig.field(sb, "isTipped", isTipped).append(',');
        AprilTagConfig.field(sb, "pitch", hivePitch, 1).append(',');
        AprilTagConfig.field(sb, "baselinePitchDeg", config.hiveBaselinePitchDeg, 1).append(',');
        AprilTagConfig.field(sb, "thresholdDeg", config.hiveTippedThresholdDeg, 1).append(',');
        AprilTagConfig.field(sb, "visible", nearest != null);
        sb.append("},");

        sb.append("\"detections\":").append(shown.toString());
        sb.append('}');
        return sb.toString();
    }

    /**
     * The dashboard's "why is there no target" line, worded exactly as AprilTagTargeting's, so
     * the simulator explains itself the same way the robot does.
     */
    private static String noTargetWhy(JSONArray dets, JSONObject nearest) {
        if (dets == null || dets.length() == 0 || nearest == null) {
            return "no tags in view";
        }
        String name = nearest.optString("name", "");
        if (name.isEmpty() || "null".equals(name)) {
            name = "tag " + nearest.optInt("id", -1);
        }
        double roll = nearest.optDouble("roll", 0);
        if (Math.abs(roll) > config.maxRollDeg) {
            return String.format(Locale.US, "%s: roll %.0f deg is past the %.0f deg limit - "
                    + "the cell looks upside down", name, roll, config.maxRollDeg);
        }
        if ("cluster".equals(nearest.optString("kind", "single"))) {
            int percent = nearest.optInt("percent", 100);
            if (percent < config.minClusterPercent) {
                return String.format(Locale.US, "%s: %d%% of the cluster visible, min is %d%%",
                        name, percent, config.minClusterPercent);
            }
            if (!allianceOk(name)) {
                return name + ": " + allianceMismatch(name);
            }
        } else {
            double margin = nearest.optDouble("margin", 100);
            if (margin < config.minDecisionMargin) {
                return String.format(Locale.US, "%s: decision margin %.0f, min is %.0f",
                        name, margin, config.minDecisionMargin);
            }
            if (!allianceOk(name)) {
                return name + ": " + allianceMismatch(name);
            }
        }
        return name + ": nothing scorable, and no rule explains it - report this";
    }

    private static String allianceMismatch(String name) {
        char hint = Character.toUpperCase(name.charAt(0));
        return config.alliance.toLowerCase(Locale.US) + " alliance filter, this one is "
                + (hint == 'R' ? "red" : "blue");
    }

    /** Same two rules as AprilTagTargeting, applied to the sim's simple JSON detections. */
    private static boolean isScorable(JSONObject d) {
        double roll = d.optDouble("roll", 0);
        if (Math.abs(roll) > config.maxRollDeg) {
            return false;
        }
        String kind = d.optString("kind", "single");
        String name = d.optString("name", "");
        int percent = d.optInt("percent", 100);
        double margin = d.optDouble("margin", 100);
        if ("cluster".equals(kind)) {
            if (percent < config.minClusterPercent) {
                return false;
            }
            return allianceOk(name);
        }
        return margin >= config.minDecisionMargin;
    }

    private static boolean allianceOk(String name) {
        if ("ANY".equals(config.alliance) || name == null || name.isEmpty()) {
            return true;
        }
        char hint = Character.toUpperCase(name.charAt(0));
        if (hint != 'R' && hint != 'B') {
            return true;
        }
        return hint == config.alliance.charAt(0);
    }

    private static JSONArray parseIngest() {
        try {
            JSONObject wrapper = new JSONObject(ingestPayload);
            return wrapper.optJSONArray("detections");
        } catch (Exception e) {
            return new JSONArray();
        }
    }

    /**
     * Animated fake BIOBUZZ field: an upright scorable RED cluster drifting past, an
     * upside-down one (roll ~155), a BLUE cluster (wrong alliance), and a stray single tag.
     */
    private static JSONArray syntheticDetections() {
        double t = System.currentTimeMillis() / 1000.0;
        JSONArray a = new JSONArray();

        a.put(cluster("RED AUDIENCE", 100,
                -20 + 35 * Math.sin(t / 4.0),           // bearing sweeps left/right
                30 + 12 * Math.sin(t / 6.0),           // range breathes
                8 * Math.sin(t / 5.0),                 // elevation drifts
                12 * Math.sin(t / 3.0)));              // roll stays upright -> scorable

        a.put(cluster("RED SCORING", 50,
                35 + 10 * Math.sin(t / 7.0),
                55 + 15 * Math.cos(t / 5.0),
                -12 + 6 * Math.cos(t / 4.0),
                155 + 8 * Math.sin(t / 8.0)));         // upside down -> unscorable

        a.put(cluster("BLUE AUDIENCE", 100,
                -40 + 8 * Math.sin(t / 5.5),
                42 + 10 * Math.cos(t / 6.5),
                5 * Math.sin(t / 4.5),
                -15 + 10 * Math.sin(t / 4.0)));        // upright but wrong alliance

        if (Math.sin(t / 2.5) > 0.2) {                 // pops in and out
            JSONObject single = new JSONObject();
            single.put("kind", "single");
            single.put("name", JSONObject.NULL);
            single.put("id", 4);
            single.put("meta", false);
            single.put("percent", 100);
            single.put("margin", 70 + 25 * Math.sin(t));
            single.put("hamming", 0);
            poseInto(single, 15 * Math.sin(t / 3.5), 18 + 5 * Math.sin(t / 2.0),
                    3 * Math.sin(t / 4.0), 5 * Math.sin(t / 2.2));
            a.put(single);
        }
        return a;
    }

    private static JSONObject cluster(String name, int percent, double bearingDeg,
                                      double rangeIn, double elevDeg, double rollDeg) {
        JSONObject d = new JSONObject();
        d.put("kind", "cluster");
        d.put("name", name);
        d.put("id", -1);
        d.put("meta", true);
        d.put("percent", percent);
        d.put("margin", 100);
        d.put("hamming", 0);
        poseInto(d, bearingDeg, rangeIn, elevDeg, rollDeg);
        return d;
    }

    private static void poseInto(JSONObject d, double bearingDeg, double rangeIn,
                                 double elevDeg, double rollDeg) {
        double br = Math.toRadians(bearingDeg);
        double el = Math.toRadians(elevDeg);
        d.put("bearing", round1(bearingDeg));
        d.put("elevation", round1(elevDeg));
        d.put("range", round1(rangeIn));
        d.put("x", round1(rangeIn * Math.sin(br)));
        d.put("y", round1(rangeIn * Math.sin(el)));
        d.put("z", round1(rangeIn * Math.cos(br) * Math.cos(el)));
        d.put("pitch", round1(-elevDeg));
        d.put("roll", round1(rollDeg));
        d.put("yaw", round1(bearingDeg));
    }

    private static double round1(double v) {
        return Math.round(v * 10.0) / 10.0;
    }

    private static String readBody(InputStream in) throws IOException {
        ByteArrayOutputStream out = new ByteArrayOutputStream(2048);
        byte[] buf = new byte[1024];
        int n;
        while ((n = in.read(buf)) > 0) {
            out.write(buf, 0, n);
        }
        return new String(out.toByteArray(), StandardCharsets.UTF_8);
    }

    private static void respond(HttpExchange exchange, int code, String body) throws IOException {
        byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
        exchange.getResponseHeaders().set("Content-Type", "application/json; charset=utf-8");
        exchange.sendResponseHeaders(code, bytes.length);
        OutputStream os = exchange.getResponseBody();
        os.write(bytes);
        os.flush();
    }
}
