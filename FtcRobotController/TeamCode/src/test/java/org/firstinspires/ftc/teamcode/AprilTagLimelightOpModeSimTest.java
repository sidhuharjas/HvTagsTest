package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeLimelight3A;
import org.horizon36596.simloop.opmode.OpModeRunResult;
import org.horizon36596.simloop.opmode.OpModeRunner;
import org.horizon36596.simloop.vision.AprilTagLayout;
import org.horizon36596.simloop.vision.CameraMount;
import org.json.JSONObject;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;
import org.opencv.core.Mat;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * The shipping OpMode on the Limelight source, driven through SimLoop.
 *
 * <p>{@link FakeLimelight3A} is registered in the hardware map under the name the config asks
 * for, so {@code init()} takes the branch a robot with a Limelight takes: no VisionPortal, no
 * OpenCV, just {@code getLatestResult()} every loop and {@link LimelightBridge} turning it into
 * the same detection list the webcam path produces. Two runs are covered - starting on the
 * Limelight, and flipping the dashboard's source selector while the OpMode is already running -
 * because both go through the same {@code switchSource()} on the loop thread.
 *
 * <p>The geometry is the one {@link LimelightBridgeTest} checks in isolation: two tags ahead of
 * a camera at (6, 0, 10). Only the seam that matters is overridden here: {@code createConfig()}
 * picks the starting source (a test cannot POST before init exists), {@code createProcessor()}
 * and {@code rebuildPortal()} stay stubbed because the webcam half cannot run on a plain JVM -
 * which is exactly the half the Limelight branch must not touch.
 */
class AprilTagLimelightOpModeSimTest {

    /** A processor that owns no native code and never reports a detection. */
    private static final class StubProcessor extends AprilTagProcessor {
        @Override
        public void init(int width, int height, CameraCalibration calibration) {
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasToHttps, Object userContext) {
        }

        @Override
        public void setDecimation(float decimation) {
        }

        @Override
        public void setPoseSolver(PoseSolver poseSolver) {
        }

        @Override
        public int getPerTagAvgPoseSolveTime() {
            return 0;
        }

        @Override
        public ArrayList<AprilTagDetection> getDetections() {
            return new ArrayList<>();
        }

        @Override
        public ArrayList<AprilTagDetection> getFreshDetections() {
            return new ArrayList<>();
        }
    }

    /** The shipping OpMode with the two Android/native pieces taken out, starting from {@code start}. */
    private static class SimOpMode extends AprilTagDashboardOpMode {
        private final AprilTagConfig start;

        private SimOpMode() {
            this(new AprilTagConfig());
        }

        private SimOpMode(AprilTagConfig start) {
            this.start = start;
        }

        @Override
        protected AprilTagConfig createConfig() {
            return start;
        }

        @Override
        protected AprilTagProcessor createProcessor() {
            return new StubProcessor();
        }

        @Override
        protected void rebuildPortal() {
            // No VisionPortal on a plain JVM: in webcam mode the dashboard runs against a closed camera.
        }
    }

    /**
     * The same run with the two Limelight REST calls answered in-process: the pipeline file the
     * button reads, and the upload it makes afterwards. The device itself stays
     * {@link FakeLimelight3A}, so the status - slot and pipeline type - is still its own.
     */
    private static final class SyncOpMode extends SimOpMode {
        volatile String uploadedJson;
        volatile Integer uploadedSlot;
        private final String pipelineFile;

        private SyncOpMode(AprilTagConfig start, String pipelineFile) {
            super(start);
            this.pipelineFile = pipelineFile;
        }

        @Override
        protected String httpGetJson(String url) {
            return pipelineFile;
        }

        @Override
        protected boolean uploadPipelineJson(String json, int slot) {
            uploadedJson = json;
            uploadedSlot = slot;
            return true;
        }
    }

    /** Two tags ahead of a camera 6 in forward, 10 in up, robot at the field origin facing +X. */
    private static FakeLimelight3A registerLimelight(FakeHardwareMap hardwareMap) {
        AprilTagLayout layout = AprilTagLayout.builder()
                .addTag(1, 70.0, 10.0, 6.0, Math.PI, 5.0)
                .addTag(2, 70.0, -10.0, 6.0, Math.PI, 5.0)
                .build();
        FakeLimelight3A device = FakeLimelight3A.builder(layout,
                CameraMount.front(6.0, 10.0, 0.0), 65.0, 51.0).build();
        device.setRobotPose(0.0, 0.0, 0.0);
        hardwareMap.register("limelight", device);
        return device;
    }

    private static boolean portFree(int port) {
        try (ServerSocket probe = new ServerSocket()) {
            probe.setReuseAddress(true);
            probe.bind(new InetSocketAddress(port));
            return true;
        } catch (IOException e) {
            return false;
        }
    }

    /** GET /api/state. Body in the return value; status in status[0] (-1 on a transport error). */
    private static String fetchState(int[] status) {
        HttpURLConnection conn = null;
        try {
            conn = (HttpURLConnection) new URL("http://127.0.0.1:"
                    + AprilTagDashboardServer.PORT + "/api/state").openConnection();
            conn.setConnectTimeout(500);
            conn.setReadTimeout(500);
            status[0] = conn.getResponseCode();
            InputStream in = status[0] >= 400 ? conn.getErrorStream() : conn.getInputStream();
            if (in == null) {
                return "";
            }
            StringBuilder sb = new StringBuilder(4096);
            byte[] buf = new byte[4096];
            int n;
            while ((n = in.read(buf)) != -1) {
                sb.append(new String(buf, 0, n, StandardCharsets.UTF_8));
            }
            return sb.toString();
        } catch (IOException e) {
            status[0] = -1;
            return "";
        } finally {
            if (conn != null) {
                conn.disconnect();
            }
        }
    }

    /** POST /api/config, the body the dashboard's controls send. Returns the HTTP status. */
    private static int postConfig(String body) {
        HttpURLConnection conn = null;
        try {
            conn = (HttpURLConnection) new URL("http://127.0.0.1:"
                    + AprilTagDashboardServer.PORT + "/api/config").openConnection();
            conn.setRequestMethod("POST");
            conn.setConnectTimeout(1000);
            conn.setReadTimeout(1000);
            conn.setDoOutput(true);
            conn.setRequestProperty("Content-Type", "application/json");
            try (OutputStream out = conn.getOutputStream()) {
                out.write(body.getBytes(StandardCharsets.UTF_8));
            }
            return conn.getResponseCode();
        } catch (IOException e) {
            return -1;
        } finally {
            if (conn != null) {
                conn.disconnect();
            }
        }
    }

    @Test
    void initTakesTheLimelightBranchAndTheLoopReadsItsResult() {
        Assumptions.assumeTrue(portFree(AprilTagDashboardServer.PORT),
                "port " + AprilTagDashboardServer.PORT + " is in use - stop the desktop "
                        + "dashboard (DesktopSimMain) before running these tests");

        FakeHardwareMap hardwareMap = new FakeHardwareMap();
        registerLimelight(hardwareMap);

        AprilTagConfig start = new AprilTagConfig();
        start.cameraMode = AprilTagConfig.MODE_LIMELIGHT;
        start.tagNames = "{\"1\":\"R_HIVE\"}";
        AprilTagDashboardOpMode opMode = new SimOpMode(start);

        int[] lastStatus = new int[] {-1};
        String[] body = new String[1];

        OpModeRunResult result = new OpModeRunner(opMode, hardwareMap)
                .initLoopTicks(2)
                .activeTicks(12)
                .eachTick(dt -> {
                    String json = fetchState(lastStatus);
                    if (json.contains("camState")) {
                        body[0] = json;
                    }
                })
                .run();

        assertTrue(result.ticks() > 0, "the run completed ticks");
        assertNotNull(body[0], "no state JSON was served; status=" + lastStatus[0]);

        assertTrue(body[0].contains("\"LIMELIGHT:OK\""),
                "the camera reads the Limelight, not a VisionPortal state: " + body[0]);
        assertTrue(body[0].contains("\"llIp\":\"sim\""),
                "the state carries the device the dashboard links to: " + body[0]);
        assertTrue(body[0].contains("\"streamUrl\":\"/stream.mjpg\""),
                "the live view is offered in Limelight mode too, not just on the webcam: "
                        + body[0]);
        assertTrue(body[0].contains("\"name\":\"R_HIVE\""),
                "a fiducial arrived, named from tagNames: " + body[0]);
        assertTrue(body[0].contains("\"range\":64.9"),
                "the range the bridge computed from the payload's metres: " + body[0]);
        assertTrue(body[0].contains("\"scorable\":true"),
                "targeting ran on the Limelight detections: " + body[0]);
        assertFalse(body[0].contains("\"CLOSED\""),
                "the webcam branch must not have run: " + body[0]);

        assertFalse(result.telemetry().transmissions().isEmpty(),
                "the Driver Station sees telemetry from the Limelight run");

        assertTrue(portFree(AprilTagDashboardServer.PORT),
                "stop() must close the dashboard, or the next run cannot bind 8080");
    }

    @Test
    void theDashboardSwitchesSourceLiveWhileTheOpModeRuns() {
        Assumptions.assumeTrue(portFree(AprilTagDashboardServer.PORT),
                "port " + AprilTagDashboardServer.PORT + " is in use - stop the desktop "
                        + "dashboard (DesktopSimMain) before running these tests");

        FakeHardwareMap hardwareMap = new FakeHardwareMap();
        registerLimelight(hardwareMap);

        // Webcam config, exactly what a team starts with: no Limelight until the POST asks.
        AprilTagDashboardOpMode opMode = new SimOpMode();

        int[] lastStatus = new int[] {-1};
        boolean[] posted = {false};
        boolean[] backToWebcam = {false};
        boolean[] sawLimelight = {false};
        boolean[] sawDetections = {false};
        String[] lastBody = {""};

        new OpModeRunner(opMode, hardwareMap)
                .initLoopTicks(2)
                .activeTicks(20)
                .eachTick(dt -> {
                    String json = fetchState(lastStatus);
                    if (json.isEmpty()) {
                        return;
                    }
                    lastBody[0] = json;
                    if (!posted[0] && json.contains("camState")) {
                        posted[0] = postConfig("{\"cameraMode\":\"LIMELIGHT\","
                                + "\"tagNames\":\"{\\\"1\\\":\\\"R_HIVE\\\"}\"}") == 200;
                        return;
                    }
                    if (posted[0] && json.contains("LIMELIGHT:OK")) {
                        sawLimelight[0] = true;
                        if (json.contains("\"name\":\"R_HIVE\"")) {
                            sawDetections[0] = true;
                        }
                    }
                    if (sawDetections[0] && !backToWebcam[0]) {
                        backToWebcam[0] = postConfig("{\"cameraMode\":\"WEBCAM\"}") == 200;
                    }
                })
                .run();

        assertTrue(posted[0], "the config POST must reach the OpMode; status=" + lastStatus[0]);
        assertTrue(sawLimelight[0],
                "switching the source must take the Limelight branch without a restart: "
                        + lastBody[0]);
        assertTrue(sawDetections[0],
                "the switched source must produce detections, not an empty list: " + lastBody[0]);
        assertTrue(backToWebcam[0], "the run must be able to switch back");

        assertFalse(lastBody[0].contains("LIMELIGHT:"),
                "after switching back the camera reads the webcam state again: " + lastBody[0]);
        assertTrue(lastBody[0].contains("\"cameraMode\":\"WEBCAM\""),
                "the config the dashboard reads back is the one it posted: " + lastBody[0]);
        assertTrue(portFree(AprilTagDashboardServer.PORT), "stop() must give 8080 back");
    }

    /**
     * The whole "push tag size" round trip: a dashboard POST, the loop thread reading the
     * device's pipeline file, the patch, the upload, and the sentence that comes back.
     *
     * <p>The pipeline file is the shape the button has to survive - a millimetre tag size beside
     * settings that must not move - and the upload is captured, so what reaches the camera is
     * checked key by key rather than trusted.
     */
    @Test
    void thePushButtonReadsPatchesAndUploadsTheLimelightsOwnPipeline() throws Exception {
        Assumptions.assumeTrue(portFree(AprilTagDashboardServer.PORT),
                "port " + AprilTagDashboardServer.PORT + " is in use - stop the desktop "
                        + "dashboard (DesktopSimMain) before running these tests");

        FakeHardwareMap hardwareMap = new FakeHardwareMap();
        registerLimelight(hardwareMap);

        AprilTagConfig start = new AprilTagConfig();
        start.cameraMode = AprilTagConfig.MODE_LIMELIGHT;
        start.tagSizeIn = 6.0;
        start.applyJson("{\"limelightHost\":\"https://hive-cam.local:5807/\"}");
        SyncOpMode opMode = new SyncOpMode(start,
                "{\"tagSize\":127.0,\"fam\":\"apriltag3_36h11_classic\",\"exposure\":5,"
                        + "\"type\":\"pipe_fiducial\"}");

        int[] lastStatus = new int[] {-1};
        boolean[] posted = {false};
        String[] lastBody = {""};

        new OpModeRunner(opMode, hardwareMap)
                .initLoopTicks(2)
                .activeTicks(30)
                .eachTick(dt -> {
                    String json = fetchState(lastStatus);
                    if (json.isEmpty()) {
                        return;
                    }
                    lastBody[0] = json;
                    if (!posted[0] && json.contains("camState")) {
                        posted[0] = postConfig("{\"pushTagSize\":true}") == 200;
                    }
                })
                .run();

        assertTrue(posted[0], "the button's POST must reach the OpMode; status=" + lastStatus[0]);
        assertNotNull(opMode.uploadedJson,
                "the patched pipeline must be uploaded: " + lastBody[0]);
        assertEquals(0, opMode.uploadedSlot.intValue(),
                "the slot the device reports as active is the one written");

        JSONObject uploaded = new JSONObject(opMode.uploadedJson);
        assertEquals(152.4, uploaded.getDouble("tagSize"), 1e-9,
                "6.0 in, written in the millimetres the file was already using");
        assertEquals("apriltag3_36h11_classic", uploaded.getString("fam"),
                "every other setting rides along untouched");
        assertEquals(5, uploaded.getInt("exposure"));
        assertEquals("pipe_fiducial", uploaded.getString("type"));

        assertTrue(lastBody[0].contains("\"llSync\""),
                "the outcome is published for the dashboard: " + lastBody[0]);
        assertTrue(lastBody[0].contains("uploaded"),
                "and it reads as a success: " + lastBody[0]);
        assertTrue(lastBody[0].contains("\"llPipe\":\"0/pipe_fiducial\""),
                "the dashboard also sees which pipeline the device is running: " + lastBody[0]);

        assertEquals("http://hive-cam.local:5807", opMode.llRestBase(),
                "the host typed on the dashboard decides where the REST call goes");

        assertTrue(portFree(AprilTagDashboardServer.PORT), "stop() must give 8080 back");
    }

    /** When the pipeline file gives no confident answer, nothing is written and the reason is. */
    @Test
    void aPipelineItCannotUnderstandIsRefusedAndNothingIsUploaded() {
        Assumptions.assumeTrue(portFree(AprilTagDashboardServer.PORT),
                "port " + AprilTagDashboardServer.PORT + " is in use - stop the desktop "
                        + "dashboard (DesktopSimMain) before running these tests");

        FakeHardwareMap hardwareMap = new FakeHardwareMap();
        registerLimelight(hardwareMap);

        AprilTagConfig start = new AprilTagConfig();
        start.cameraMode = AprilTagConfig.MODE_LIMELIGHT;
        SyncOpMode opMode = new SyncOpMode(start, "{\"color\":\"blue\",\"exposure\":5}");

        int[] lastStatus = new int[] {-1};
        boolean[] posted = {false};
        String[] lastBody = {""};

        new OpModeRunner(opMode, hardwareMap)
                .initLoopTicks(2)
                .activeTicks(30)
                .eachTick(dt -> {
                    String json = fetchState(lastStatus);
                    if (json.isEmpty()) {
                        return;
                    }
                    lastBody[0] = json;
                    if (!posted[0] && json.contains("camState")) {
                        posted[0] = postConfig("{\"pushTagSize\":true}") == 200;
                    }
                })
                .run();

        assertTrue(posted[0], "the button's POST must reach the OpMode; status=" + lastStatus[0]);
        assertNull(opMode.uploadedJson,
                "a file with no size-like key must never be uploaded: " + lastBody[0]);
        assertTrue(lastBody[0].contains("no size-like key"),
                "the refusal, with what the file does have, reaches the dashboard: " + lastBody[0]);
        assertTrue(portFree(AprilTagDashboardServer.PORT), "stop() must give 8080 back");
    }
}
