package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.Field;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Webcam AprilTag detection with a live tuning dashboard.
 *
 * Run this OpMode, then open the dashboard on the pit laptop:
 *   http://<robot-ip>:8080          (robot and laptop on the same wifi)
 *   http://localhost:8080           (after: adb reverse tcp:8080 tcp:8080 over USB)
 *
 * The dashboard streams detections as JSON and lets you tune exposure, gain, decimation,
 * pose solver, resolution, and the cluster-targeting rules from the FTC AprilTag Clusters
 * tech tip (alliance colour + |roll| < threshold). Everything applies live - no redeploy.
 *
 * Requires FTC SDK 12.0+ (AprilTag clusters).
 */
@TeleOp(name = "AprilTag Dashboard", group = "Vision")
public class AprilTagDashboardOpMode extends OpMode implements AprilTagDashboardServer.Backend {

    private AprilTagConfig config;
    private AprilTagProcessor aprilTag;
    /** Turns the portal's frames into JPEG for the dashboard's Live view. */
    private MjpegFrameGrabber frameGrabber;
    private VisionPortal portal;
    /** Non-null while the Limelight is the source: polled directly, no VisionPortal. */
    private Limelight3A limelight;
    private String lastCameraMode = AprilTagConfig.MODE_WEBCAM;
    private int lastPollHzApplied = -1;
    private AprilTagDashboardServer server;
    private List<AprilTagDetection> latestDetections = java.util.Collections.emptyList();

    private volatile String latestState = "{\"camState\":\"INIT\",\"detections\":[],\"config\":null}";
    private volatile String lastError = "";
    /** Last outcome of the dashboard's "push tag size" button, for the dashboard to display. */
    private volatile String llSync = "";
    /** The device's active pipeline, e.g. "3/pipe_fiducial", read from the last result. */
    private LLResult lastLLResult;

    private long loopMs;
    private long lastLoopNanos;
    private int lastResolution = -1;
    private String lastCameraName = "";
    private boolean lastLiveView = true;
    private boolean pendingCameraApply = true;
    private long lastExposureApplied = Long.MIN_VALUE;
    private int lastGainApplied = Integer.MIN_VALUE;
    private long cachedExpMin = 0;
    private long cachedExpMax = 0;
    private int cachedGainMin = 0;
    private int cachedGainMax = 0;
    private boolean cameraControlsProbed = false;

    @Override
    public void init() {
        config = createConfig();
        lastCameraMode = config.cameraMode;
        if (config.isLimelight()) {
            startLimelight();
        } else {
            aprilTag = createProcessor();
            rebuildPortal();
        }

        try {
            server = new AprilTagDashboardServer(this);
            String where = "Dashboard: http://" + AprilTagDashboardServer.firstLanIp()
                    + ":" + server.port();
            if (server.port() != AprilTagDashboardServer.PORT) {
                where += "  (port " + AprilTagDashboardServer.PORT + " was taken - on a Control Hub"
                        + " that is OnBot Java)";
            }
            telemetry.addLine(where);
        } catch (IOException e) {
            lastError = "dashboard server failed: " + e;
            telemetry.addLine(lastError);
        }
        // The portal is built before the server exists, so the grabber learns where to send
        // frames here, once there is somewhere to send them.
        if (frameGrabber != null) {
            frameGrabber.setServer(server);
        }
    }

    @Override
    public void init_loop() {
        try {
            latestState = buildStateJson();
        } catch (Throwable e) {
            lastError = describeFailure("state", e);
        }
    }

    @Override
    public void loop() {
        try {
            loopOnce();
        } catch (Throwable e) {
            // A webcam that dies mid-stream, a control it refuses, or a missing class from a
            // mismatched SDK must not end the run. Throwable, not Exception: NoClassDefFoundError
            // and friends are Errors, and they are exactly what killed the OpMode on the field.
            lastError = describeFailure("loop", e);
        }
    }

    /**
     * "loop: java.lang.NoClassDefFoundError: ... &lt;- AprilTagTargeting.isScorable:31 &lt;- ..."
     *
     * <p>The class and the throw site are what identify the fault; the message alone often is
     * not enough (an Error frequently carries none).
     */
    private static String describeFailure(String where, Throwable e) {
        StringBuilder sb = new StringBuilder(where).append(": ").append(e);
        StackTraceElement[] frames = e.getStackTrace();
        for (int i = 0; frames != null && i < frames.length && i < 3; i++) {
            StackTraceElement f = frames[i];
            sb.append(" <- ").append(shortClass(f.getClassName()))
                    .append('.').append(f.getMethodName()).append(':').append(f.getLineNumber());
        }
        Throwable cause = e.getCause();
        if (cause != null && cause != e) {
            sb.append(" (caused by ").append(cause).append(')');
        }
        return sb.toString();
    }

    private static String shortClass(String className) {
        if (className == null) {
            return "?";
        }
        int dot = className.lastIndexOf('.');
        return dot < 0 ? className : className.substring(dot + 1);
    }

    private void loopOnce() {
        long now = System.nanoTime();
        if (lastLoopNanos != 0) {
            loopMs = (now - lastLoopNanos) / 1_000_000;
        }
        lastLoopNanos = now;

        if (config.consumeDirty()) {
            applyToVision();
            applyLimelightSettings();
            if (!config.cameraMode.equals(lastCameraMode)) {
                switchSource(); // webcam <-> Limelight, live, from the dashboard
            }
            if (config.consumePushTagSize()) {
                syncTagSizeToLimelight(); // one HTTP round-trip, on the loop thread
            }
        }
        // Camera controls only exist once the stream is actually up.
        if (pendingCameraApply && portal != null
                && portal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            applyCameraControls();
            pendingCameraApply = false;
        }

        latestDetections = fetchDetections();
        latestState = buildStateJson();

        telemetry.addData("cam", camStateText());
        telemetry.addData("fps", currentFps());
        telemetry.addData("loop ms", loopMs);
        telemetry.addData("tags seen", latestDetections.size());
        AprilTagDetection target = null;
        try {
            target = AprilTagTargeting.bestTarget(latestDetections, config);
            if (target != null) {
                telemetry.addLine("TARGET " + describe(target));
                telemetry.addData("range (in)",
                        String.format(Locale.US, "%.1f", target.ftcPose.range));
                telemetry.addData("bearing (deg)",
                        String.format(Locale.US, "%.1f", target.ftcPose.bearing));
            } else {
                telemetry.addLine("no scorable target - "
                        + AprilTagTargeting.noTargetReason(latestDetections, config));
            }
            for (AprilTagDetection d : latestDetections) {
                telemetry.addLine((AprilTagTargeting.isScorable(d, config) ? "[ok] " : "[  ] ")
                        + describe(d));
            }
        } catch (Throwable e) {
            // The Driver Station readout is a nicety; it must never be what ends the run.
            lastError = describeFailure("telemetry", e);
            telemetry.addLine("telemetry readout failed - " + e);
        }
        if (!lastError.isEmpty()) {
            telemetry.addLine(lastError);
        }
        if (!llSync.isEmpty()) {
            telemetry.addLine("push tag size: " + llSync);
        }
    }

    /**
     * The one call that touches the camera stack every loop. A USB webcam pulling out mid-match
     * throws from here, so the failure is recorded and the dashboard keeps being served.
     */
    private List<AprilTagDetection> fetchDetections() {
        try {
            if (config.isLimelight()) {
                lastLLResult = limelight == null ? null : limelight.getLatestResult();
                return LimelightBridge.toDetections(lastLLResult, config);
            }
            lastLLResult = null;
            return aprilTag == null
                    ? java.util.Collections.emptyList()
                    : new java.util.ArrayList<>(aprilTag.getDetections());
        } catch (Throwable e) {
            // Throwable: a tag library or bridge class missing from the running SDK surfaces as
            // an Error the first time a detection is actually read.
            lastError = describeFailure("camera", e);
            return java.util.Collections.emptyList();
        }
    }

    @Override
    public void stop() {
        if (server != null) {
            server.close();
            server = null;
        }
        if (frameGrabber != null) {
            frameGrabber.setServer(null);
        }
        if (portal != null) {
            portal.close();
            portal = null;
        }
        if (limelight != null) {
            try {
                limelight.stop();
            } catch (Exception ignored) {
            }
            try {
                limelight.close();
            } catch (Exception ignored) {
            }
            limelight = null;
        }
    }

    /* ---------------- AprilTagDashboardServer.Backend ---------------- */

    @Override
    public String snapshotJson() {
        return latestState;
    }

    @Override
    public void applyConfig(String jsonBody) {
        config.applyJson(jsonBody); // applied on the loop thread via consumeDirty()
    }

    /* ---------------- vision plumbing ---------------- */

    /**
     * The OpMode's own starting configuration. Shipping code always wants the defaults; a test
     * overrides this to begin on the Limelight source instead of trying to flip the dashboard
     * before {@link #init()} has even run. Everything after this call is the shipping path.
     */
    protected AprilTagConfig createConfig() {
        return new AprilTagConfig();
    }

    /**
     * Builds the AprilTag processor.
     *
     * <p>Split out of {@link #init()} so a JVM test (SimLoop) can substitute a stub:
     * {@code easyCreateWithDefaults()} reaches OpenCV's native library, which only exists on
     * the robot. The dashboard, targeting rules and telemetry do not need it, so the OpMode
     * still runs - and still reports - without a processor.
     */
    protected AprilTagProcessor createProcessor() {
        return new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(false)
                .setDrawTagID(false)
                .build();
    }

    protected void rebuildPortal() {
        if (frameGrabber == null) {
            frameGrabber = new MjpegFrameGrabber();
        }
        frameGrabber.setServer(server);
        if (portal != null) {
            try {
                portal.close();
            } catch (Exception ignored) {
            }
        }
        VisionPortal.Builder builder = new VisionPortal.Builder();
        if ("BACK".equalsIgnoreCase(config.cameraName) || "FRONT".equalsIgnoreCase(config.cameraName)) {
            // Built-in device camera - works on an emulator / RC phone with no USB webcam.
            builder.setCamera(BuiltinCameraDirection.valueOf(config.cameraName.toUpperCase(Locale.US)));
        } else {
            try {
                builder.setCamera(hardwareMap.get(WebcamName.class, config.cameraName));
            } catch (Exception e) {
                lastError = "webcam '" + config.cameraName + "' not found (" + e + "); using BACK camera";
                telemetry.addLine(lastError);
                builder.setCamera(BuiltinCameraDirection.BACK);
            }
        }
        portal = builder
                .setCameraResolution(new Size(
                        AprilTagConfig.RES_W[config.resolution],
                        AprilTagConfig.RES_H[config.resolution]))
                .enableLiveView(config.liveView)
                // The grabber goes first so it encodes the untouched frame, not one the
                // AprilTag processor has already converted in place.
                .addProcessor(frameGrabber)
                .addProcessor(aprilTag)
                .build();

        lastResolution = config.resolution;
        lastCameraName = config.cameraName;
        lastLiveView = config.liveView;
        pendingCameraApply = true;
        lastExposureApplied = Long.MIN_VALUE;
        lastGainApplied = Integer.MIN_VALUE;
        cameraControlsProbed = false;
    }

    /** Limelight half of the vision plumbing: the module detects for us, so no VisionPortal. */
    private void startLimelight() {
        try {
            limelight = hardwareMap.get(Limelight3A.class, config.limelightName);
            lastPollHzApplied = config.limelightPollHz;
            limelight.setPollRateHz(lastPollHzApplied);
            limelight.start();
            String where = limelight.getConnectionInfo();
            telemetry.addLine("Limelight: " + config.limelightName
                    + (where == null || where.isEmpty() ? "" : " @ " + where));
        } catch (Exception e) {
            limelight = null;
            lastError = "limelight '" + config.limelightName + "' not found (" + e + ")";
            telemetry.addLine(lastError);
        }
    }

    /** Dashboard changes the Limelight owns (poll rate for now) get pushed on the loop thread. */
    private void applyLimelightSettings() {
        if (limelight == null) {
            return;
        }
        if (config.limelightPollHz != lastPollHzApplied) {
            lastPollHzApplied = config.limelightPollHz;
            limelight.setPollRateHz(lastPollHzApplied);
        }
    }

    /**
     * The dashboard's "push tag size" button: read the Limelight's own pipeline file, rewrite
     * only its tag size to match {@code config.tagSizeIn}, and put the file back.
     *
     * <p>Everything else the camera has configured is carried through untouched, and every
     * refusal is a sentence the dashboard can show: a non-AprilTag pipeline, an address that
     * cannot be reached, a key that does not look like a tag size. Nothing here guesses - when
     * the pipeline file does not give a confident answer, nothing is written.
     */
    protected void syncTagSizeToLimelight() {
        try {
            if (limelight == null || !config.isLimelight()) {
                llSync = "not in Limelight mode";
                return;
            }
            if (!limelight.isConnected()) {
                llSync = "the Limelight is not responding";
                return;
            }
            LLStatus status = limelight.getStatus();
            int slot = status == null ? -1 : status.getPipelineIndex();
            String type = status == null ? null : status.getPipelineType();
            if (slot < 0) {
                llSync = "no pipeline status from the Limelight";
                return;
            }
            if (type != null && !type.isEmpty() && !"pipe_fiducial".equals(type)) {
                llSync = "pipeline " + slot + " is " + type + ", not AprilTag - select an "
                        + "AprilTag pipeline in the Limelight UI first";
                return;
            }
            String base = llRestBase();
            if (base == null) {
                llSync = "no address for the Limelight's REST API";
                return;
            }
            String url = base + "/pipeline-atindex?index=" + slot;
            String json = httpGetJson(url);
            if (json == null) {
                llSync = "no response from " + url;
                return;
            }
            LimelightBridge.PipelinePatch patch = LimelightBridge.patchTagSize(json, config.tagSizeIn);
            if (patch.json == null) {
                llSync = "pipeline " + slot + ": " + patch.message;
                return;
            }
            if (!uploadPipelineJson(patch.json, slot)) {
                llSync = "pipeline " + slot + ": the Limelight rejected the upload";
                return;
            }
            try {
                limelight.reloadPipeline();
            } catch (Exception ignored) {
                // the file is on disk either way; the next pipeline load picks it up
            }
            llSync = "pipeline " + slot + ": " + patch.message + " - uploaded";
        } catch (Exception e) {
            llSync = "sync failed: " + e;
        }
    }

    /**
     * The base URL the SDK itself polls, read from its own field so the REST call cannot drift
     * from the host the driver is talking to. A host typed into the dashboard wins first - that
     * is the point of the option, for a camera renamed away from limelight.local - and the
     * address form of {@code getConnectionInfo()} (e.g. {@code "usbnic0:172.29.1.1"}) is the
     * fallback when reflection is denied.
     */
    protected String llRestBase() {
        String host = config == null ? null : config.limelightHost;
        if (host != null && !host.isEmpty()) {
            return "http://" + host + ":5807";
        }
        if (limelight == null) {
            return null;
        }
        try {
            Field f = Limelight3A.class.getDeclaredField("baseUrl");
            f.setAccessible(true);
            Object v = f.get(limelight);
            if (v instanceof String && !((String) v).isEmpty()) {
                return (String) v;
            }
        } catch (Throwable ignored) {
            // fall through to the connection info
        }
        String info = limelightIp();
        if (info == null || info.isEmpty()) {
            return null;
        }
        Matcher ip = Pattern.compile("\\d{1,3}(\\.\\d{1,3}){3}").matcher(info);
        if (ip.find()) {
            return "http://" + ip.group() + ":5807";
        }
        if (info.indexOf(':') < 0 && info.indexOf('/') < 0) {
            return "http://" + info + ":5807"; // a bare hostname, e.g. limelight.local
        }
        return null;
    }

    /** GET of the device's pipeline file; its own method so a test can answer it. */
    protected String httpGetJson(String url) throws IOException {
        HttpURLConnection conn = (HttpURLConnection) new URL(url).openConnection();
        try {
            conn.setRequestMethod("GET");
            conn.setConnectTimeout(1000);
            conn.setReadTimeout(1500);
            if (conn.getResponseCode() != 200) {
                return null;
            }
            try (InputStream in = conn.getInputStream()) {
                ByteArrayOutputStream buf = new ByteArrayOutputStream(16 * 1024);
                byte[] chunk = new byte[8192];
                int n;
                while ((n = in.read(chunk)) != -1 && buf.size() < 256 * 1024) {
                    buf.write(chunk, 0, n);
                }
                return new String(buf.toByteArray(), java.nio.charset.StandardCharsets.UTF_8);
            }
        } finally {
            conn.disconnect();
        }
    }

    /** Upload of the patched pipeline; its own method so a test can answer it. */
    protected boolean uploadPipelineJson(String json, int slot) {
        return limelight != null && limelight.uploadPipeline(json, slot);
    }

    /**
     * Swaps the detection source live when {@code cameraMode} changes in the dashboard.
     * Teardown and setup both happen here, on the loop thread - the same place the webcam
     * path already rebuilds its portal - so neither source ever runs concurrently.
     */
    private void switchSource() {
        String from = lastCameraMode;
        String to = AprilTagConfig.normalizeMode(config.cameraMode);
        config.cameraMode = to;
        lastCameraMode = to;

        if (AprilTagConfig.MODE_LIMELIGHT.equals(to)) {
            if (portal != null) {
                try {
                    portal.close();
                } catch (Exception ignored) {
                }
                portal = null;
            }
            startLimelight();
        } else {
            if (limelight != null) {
                try {
                    limelight.stop();
                } catch (Exception ignored) {
                }
                try {
                    limelight.close();
                } catch (Exception ignored) {
                }
                limelight = null;
            }
            if (aprilTag == null) {
                aprilTag = createProcessor();
            }
            rebuildPortal();
        }
        lastError = "";
        telemetry.addLine("source " + from + " -> " + to);
    }

    /** What the dashboard and Driver Station call the camera, whichever source is live. */
    private String camStateText() {
        if (config != null && config.isLimelight()) {
            if (limelight == null) {
                return "LIMELIGHT:MISSING";
            }
            if (!limelight.isConnected()) {
                return "LIMELIGHT:OFFLINE";
            }
            long staleMs = limelight.getTimeSinceLastUpdate();
            return staleMs > 2000 ? "LIMELIGHT:STALE" : "LIMELIGHT:OK";
        }
        return portal == null ? "CLOSED" : portal.getCameraState().name();
    }

    /** Frame rate of whichever source is live: VisionPortal's, or the Limelight's own. */
    private double currentFps() {
        if (config != null && config.isLimelight()) {
            if (limelight == null || !limelight.isConnected()) {
                return 0;
            }
            LLStatus status = limelight.getStatus();
            return status == null ? 0 : status.getFps();
        }
        return portal == null ? 0 : portal.getFps();
    }

    private void applyToVision() {
        if (aprilTag != null) {
            aprilTag.setDecimation(Math.max(AprilTagConfig.DECIMATION_MIN,
                    Math.min(AprilTagConfig.DECIMATION_MAX, config.decimation)));
            try {
                AprilTagProcessor.PoseSolver solver = AprilTagProcessor.PoseSolver.valueOf(config.poseSolver);
                if (solver == AprilTagProcessor.PoseSolver.APRILTAG_BUILTIN) {
                    solver = AprilTagProcessor.PoseSolver.OPENCV_ITERATIVE;
                }
                aprilTag.setPoseSolver(solver);
            } catch (Exception e) {
                lastError = "unknown pose solver: " + config.poseSolver;
                aprilTag.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_ITERATIVE);
            }
        }

        if (portal != null
                && (config.resolution != lastResolution || !config.cameraName.equals(lastCameraName))) {
            rebuildPortal(); // resolution / camera changes need a new portal
        }

        if (portal != null && config.liveView != lastLiveView) {
            if (config.liveView) {
                portal.resumeLiveView();
            } else {
                portal.stopLiveView();
            }
            lastLiveView = config.liveView;
        }

        pendingCameraApply = true;
    }

    /** Exposure and gain need the stream to be up; called from loop() when it is. */
    private void applyCameraControls() {
        if (portal == null) {
            return;
        }
        try {
            applyExposure();
        } catch (Exception e) {
            // A webcam whose driver refuses this mode must not end the match.
            lastError = "exposure: " + e;
            lastExposureApplied = config.exposureUs;
        }
        try {
            applyGain();
        } catch (Exception e) {
            lastError = "gain: " + e;
            lastGainApplied = config.gain;
        }
    }

    private void applyExposure() {
        if (config.exposureUs == lastExposureApplied) {
            return;
        }
        ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
        if (exposure != null && exposure.isExposureSupported()) {
            if (config.exposureUs <= 0) {
                if (exposure.isModeSupported(ExposureControl.Mode.Auto)) {
                    exposure.setMode(ExposureControl.Mode.Auto);
                }
            } else if (exposure.isModeSupported(ExposureControl.Mode.Manual)) {
                exposure.setMode(ExposureControl.Mode.Manual);
                long minUs = exposure.getMinExposure(TimeUnit.MICROSECONDS);
                long maxUs = exposure.getMaxExposure(TimeUnit.MICROSECONDS);
                exposure.setExposure(Math.max(minUs, Math.min(maxUs, config.exposureUs)),
                        TimeUnit.MICROSECONDS);
            }
        }
        lastExposureApplied = config.exposureUs;
    }

    private void applyGain() {
        if (config.gain == lastGainApplied) {
            return;
        }
        if (config.gain < 0) {
            lastGainApplied = config.gain; // "leave alone" - nothing to push
            return;
        }
        GainControl gain = portal.getCameraControl(GainControl.class);
        if (gain != null) {
            gain.setGain(Math.max(gain.getMinGain(), Math.min(gain.getMaxGain(), config.gain)));
            lastGainApplied = config.gain;
        }
    }

    /* ---------------- JSON state ---------------- */

    private String limelightIp() {
        if (limelight == null) {
            return null;
        }
        try {
            return limelight.getConnectionInfo();
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * The pipeline the device is running, as the dashboard reads it: "3/pipe_fiducial".
     *
     * <p>Taken from the result the loop already fetched rather than from {@code getStatus()},
     * which is an HTTP call - the pipeline line must not cost a request of its own.
     */
    private String llPipeText() {
        LLResult r = lastLLResult;
        if (r == null || !r.isValid()) {
            return "";
        }
        String type = r.getPipelineType();
        return r.getPipelineIndex() + "/" + (type == null || type.isEmpty() ? "?" : type);
    }

    /** {@return the path the page should show as video, or "" when this source has none}. */
    private String streamUrl() {
        if (config == null || config.isLimelight()) {
            return "";
        }
        return "/stream.mjpg";
    }

    private String buildStateJson() {
        StringBuilder sb = new StringBuilder(4096);
        sb.append('{');
        AprilTagConfig.field(sb, "camState", camStateText()).append(',');
        AprilTagConfig.field(sb, "fps", currentFps(), 1).append(',');
        AprilTagConfig.field(sb, "loopMs", (double) loopMs).append(',');
        AprilTagConfig.field(sb, "ip", AprilTagDashboardServer.firstLanIp()).append(',');
        // Lets the dashboard link straight to the Limelight's own config page (:5801).
        AprilTagConfig.field(sb, "llIp", limelightIp()).append(',');
        // The "push tag size" button's outcome, and the pipeline the device is running.
        AprilTagConfig.field(sb, "llSync", llSync).append(',');
        AprilTagConfig.field(sb, "llPipe", llPipeText()).append(',');
        // Whatever last stopped a loop: shown on the dashboard so it can be read out loud.
        AprilTagConfig.field(sb, "error", lastError).append(',');
        // Where the page fetches the video for the live source; same origin as the page, so
        // the stream works over the robot's wifi and through adb reverse alike.
        AprilTagConfig.field(sb, "streamUrl", streamUrl()).append(',');

        // Live camera control ranges so the dashboard sliders can adapt to the webcam.
        // Probed once when streaming starts to prevent saturating the USB bus on every frame.
        if (!cameraControlsProbed && portal != null
                && portal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            try {
                ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
                if (exposure != null && exposure.isExposureSupported()) {
                    cachedExpMin = exposure.getMinExposure(TimeUnit.MICROSECONDS);
                    cachedExpMax = exposure.getMaxExposure(TimeUnit.MICROSECONDS);
                }
            } catch (Exception ignored) {
                // Sliders keep their defaults.
            }
            try {
                GainControl gain = portal.getCameraControl(GainControl.class);
                if (gain != null) {
                    cachedGainMin = gain.getMinGain();
                    cachedGainMax = gain.getMaxGain();
                }
            } catch (Exception ignored) {
                // Sliders keep their defaults.
            }
            cameraControlsProbed = true;
        }
        AprilTagConfig.field(sb, "expMinUs", (double) cachedExpMin).append(',');
        AprilTagConfig.field(sb, "expMaxUs", (double) cachedExpMax).append(',');
        AprilTagConfig.field(sb, "gainMin", (double) cachedGainMin).append(',');
        AprilTagConfig.field(sb, "gainMax", (double) cachedGainMax).append(',');

        sb.append("\"config\":").append(config.toJson()).append(',');

        AprilTagDetection target = null;
        String targetWhy = "";
        try {
            target = AprilTagTargeting.bestTarget(latestDetections, config);
            if (target == null) {
                targetWhy = AprilTagTargeting.noTargetReason(latestDetections, config);
            }
        } catch (Throwable e) {
            // Targeting reads the SDK's detection classes; keep the page alive without it.
            lastError = describeFailure("targeting", e);
            target = null;
            targetWhy = "targeting failed - " + e;
        }
        if (target == null) {
            sb.append("\"target\":null,");
        } else {
            sb.append("\"target\":");
            appendDetection(sb, target);
            sb.append(',');
        }
        AprilTagConfig.field(sb, "targetWhy", targetWhy).append(',');

        // BIOBUZZ hive-cell tip check (see AprilTagConfig.isTipped). Uses the NEAREST cell
        // by any means - a tipped cell is the one whose tags go unscorable, so scorable-only
        // selection would hide exactly the case this reads out.
        AprilTagDetection nearest = null;
        for (AprilTagDetection d : latestDetections) {
            // A detection with no solved pose cannot be the nearest cell; skip it, do not throw.
            if (d.ftcPose != null
                    && (nearest == null || d.ftcPose.range < nearest.ftcPose.range)) {
                nearest = d;
            }
        }
        double hivePitch = nearest == null ? Double.NaN : nearest.ftcPose.pitch;
        boolean isTipped = AprilTagConfig.isTipped(hivePitch, config.hiveBaselinePitchDeg,
                config.hiveTippedThresholdDeg);
        sb.append("\"hive\":{");
        AprilTagConfig.field(sb, "isTipped", isTipped).append(',');
        AprilTagConfig.field(sb, "pitch", hivePitch, 1).append(',');
        AprilTagConfig.field(sb, "baselinePitchDeg", config.hiveBaselinePitchDeg, 1).append(',');
        AprilTagConfig.field(sb, "thresholdDeg", config.hiveTippedThresholdDeg, 1).append(',');
        AprilTagConfig.field(sb, "visible", nearest != null);
        sb.append("},");

        sb.append("\"detections\":[");
        boolean showUnscorable = config.showUnscorable;
        boolean first = true;
        for (AprilTagDetection d : latestDetections) {
            boolean scorable;
            try {
                scorable = AprilTagTargeting.isScorable(d, config);
            } catch (Throwable e) {
                lastError = describeFailure("scorable", e);
                scorable = false;
            }
            if (!scorable && !showUnscorable) {
                continue;
            }
            if (!first) {
                sb.append(',');
            }
            first = false;
            appendDetection(sb, d);
        }
        sb.append("]}");
        return sb.toString();
    }

    /**
     * One detection as a JSON object. A tag that cannot be described (a class missing from the
     * running SDK, say) still gets a valid row, because a broken array would take the page down.
     */
    private void appendDetection(StringBuilder sb, AprilTagDetection d) {
        StringBuilder one = new StringBuilder(256);
        try {
            appendDetectionFields(one, d);
            sb.append(one);
        } catch (Throwable e) {
            lastError = describeFailure("detection", e);
            sb.append("{\"kind\":\"single\",\"name\":null,\"id\":-1,\"meta\":false,")
                    .append("\"scorable\":false,\"percent\":0,\"margin\":0,\"hamming\":0,")
                    .append("\"hasPose\":false,\"range\":null,\"bearing\":null,\"elevation\":null,")
                    .append("\"x\":null,\"y\":null,\"z\":null,\"pitch\":null,\"roll\":null,")
                    .append("\"yaw\":null}");
        }
    }

    private void appendDetectionFields(StringBuilder sb, AprilTagDetection d) {
        boolean cluster = d instanceof AprilTagClusterDetection;
        boolean single = d instanceof AprilTagSingleDetection;
        String name = null;
        int id = -1;
        int percent = 0;
        boolean meta = false;
        float margin = 0;
        int hamming = 0;

        if (cluster) {
            AprilTagClusterDetection c = (AprilTagClusterDetection) d;
            percent = c.percentClusterFound;
            meta = c.metadata != null;
            name = meta ? c.metadata.name : null;
        } else if (single) {
            AprilTagSingleDetection s = (AprilTagSingleDetection) d;
            id = s.id;
            margin = s.decisionMargin;
            hamming = s.hamming;
            meta = s.metadata != null;
            name = meta ? s.metadata.name : null;
        }

        sb.append('{');
        AprilTagConfig.field(sb, "kind", cluster ? "cluster" : "single").append(',');
        if (name == null) {
            sb.append("\"name\":null,");
        } else {
            AprilTagConfig.field(sb, "name", name).append(',');
        }
        AprilTagConfig.field(sb, "id", (double) id).append(',');
        AprilTagConfig.field(sb, "meta", meta).append(',');
        AprilTagConfig.field(sb, "scorable", AprilTagTargeting.isScorable(d, config)).append(',');
        AprilTagConfig.field(sb, "percent", (double) percent).append(',');
        AprilTagConfig.field(sb, "margin", (double) margin, 1).append(',');
        AprilTagConfig.field(sb, "hamming", (double) hamming).append(',');
        // A detection can turn up with no solved pose (no pose solver, a failed solve, a tag
        // the SDK could not localise). Sending nulls keeps the tag on the dashboard instead
        // of throwing a NullPointerException out of the state builder.
        AprilTagConfig.field(sb, "hasPose", d.ftcPose != null).append(',');
        AprilTagConfig.field(sb, "range", d.ftcPose == null ? Double.NaN : d.ftcPose.range, 1).append(',');
        AprilTagConfig.field(sb, "bearing", d.ftcPose == null ? Double.NaN : d.ftcPose.bearing, 1).append(',');
        AprilTagConfig.field(sb, "elevation", d.ftcPose == null ? Double.NaN : d.ftcPose.elevation, 1).append(',');
        AprilTagConfig.field(sb, "x", d.ftcPose == null ? Double.NaN : d.ftcPose.x, 1).append(',');
        AprilTagConfig.field(sb, "y", d.ftcPose == null ? Double.NaN : d.ftcPose.y, 1).append(',');
        AprilTagConfig.field(sb, "z", d.ftcPose == null ? Double.NaN : d.ftcPose.z, 1).append(',');
        AprilTagConfig.field(sb, "pitch", d.ftcPose == null ? Double.NaN : d.ftcPose.pitch, 1).append(',');
        AprilTagConfig.field(sb, "roll", d.ftcPose == null ? Double.NaN : d.ftcPose.roll, 1).append(',');
        AprilTagConfig.field(sb, "yaw", d.ftcPose == null ? Double.NaN : d.ftcPose.yaw, 1);
        sb.append('}');
    }

    private static String describe(AprilTagDetection d) {
        if (d instanceof AprilTagClusterDetection) {
            AprilTagClusterDetection c = (AprilTagClusterDetection) d;
            return String.format(Locale.US, "cluster %s  %d%% visible  roll %.0f deg  range %.1f in",
                    c.metadata == null ? "?" : c.metadata.name,
                    c.percentClusterFound,
                    c.ftcPose == null ? Double.NaN : c.ftcPose.roll,
                    c.ftcPose == null ? Double.NaN : c.ftcPose.range);
        }
        if (d instanceof AprilTagSingleDetection) {
            AprilTagSingleDetection s = (AprilTagSingleDetection) d;
            return String.format(Locale.US, "single ID %d (%s)  margin %.0f  range %.1f in",
                    s.id, s.metadata == null ? "no metadata" : s.metadata.name,
                    s.decisionMargin,
                    s.ftcPose == null ? Double.NaN : s.ftcPose.range);
        }
        return "unknown detection type";
    }
}
