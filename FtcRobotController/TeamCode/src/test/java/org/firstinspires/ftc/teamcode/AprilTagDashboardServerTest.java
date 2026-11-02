package org.firstinspires.ftc.teamcode;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.URL;
import java.nio.charset.StandardCharsets;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * The HTTP contract the browser dashboard speaks: {@code GET /} serves the page,
 * {@code POST /api/config} reaches the backend on the OpMode's thread, {@code GET /api/state}
 * returns what the backend last published, and {@code close()} really gives the port back.
 *
 * <p>The server prefers port 8080 but moves to 8082+ when 8080 is taken - on a Control Hub the
 * Robot Controller app serves OnBot Java there - so these tests are skipped, loudly, if that
 * whole range is held by something else.
 */
class AprilTagDashboardServerTest {

    /** Stands in for {@code AprilTagDashboardOpMode}'s two Backend methods. */
    private static final class FakeBackend implements AprilTagDashboardServer.Backend {
        volatile String state = "{\"camState\":\"STREAMING\",\"detections\":[],\"config\":null}";
        volatile String lastConfig;

        @Override
        public String snapshotJson() {
            return state;
        }

        @Override
        public void applyConfig(String jsonBody) {
            lastConfig = jsonBody;
        }
    }

    private final FakeBackend backend = new FakeBackend();
    private AprilTagDashboardServer server;

    private static boolean portFree(int port) {
        try (ServerSocket probe = new ServerSocket()) {
            probe.setReuseAddress(true);
            probe.bind(new InetSocketAddress(port));
            return true;
        } catch (IOException e) {
            return false;
        }
    }

    @BeforeEach
    void startServer() throws IOException {
        Assumptions.assumeTrue(portFree(AprilTagDashboardServer.PORT),
                "port " + AprilTagDashboardServer.PORT + " is already in use - "
                        + "stop the desktop dashboard (DesktopSimMain) before running these tests");
        server = new AprilTagDashboardServer(backend);
    }

    @AfterEach
    void stopServer() {
        if (server != null) {
            server.close();
            server = null;
        }
    }

    private HttpURLConnection connect(String method, String path) throws IOException {
        URL url = new URL("http://127.0.0.1:" + AprilTagDashboardServer.PORT + path);
        HttpURLConnection conn = (HttpURLConnection) url.openConnection();
        conn.setRequestMethod(method);
        conn.setConnectTimeout(3000);
        conn.setReadTimeout(3000);
        return conn;
    }

    private static String readAll(InputStream in) throws IOException {
        StringBuilder sb = new StringBuilder(8192);
        byte[] buf = new byte[8192];
        int n;
        while ((n = in.read(buf)) != -1) {
            sb.append(new String(buf, 0, n, StandardCharsets.UTF_8));
        }
        return sb.toString();
    }

    private String get(String path) throws IOException {
        HttpURLConnection conn = connect("GET", path);
        int code = conn.getResponseCode();
        String body = code >= 400 ? readAll(conn.getErrorStream()) : readAll(conn.getInputStream());
        assertEquals(200, code, "GET " + path + " -> " + body);
        conn.disconnect();
        return body;
    }

    private int post(String path, String body) throws IOException {
        HttpURLConnection conn = connect("POST", path);
        conn.setDoOutput(true);
        conn.setRequestProperty("Content-Type", "application/json");
        try (OutputStream out = conn.getOutputStream()) {
            out.write(body.getBytes(StandardCharsets.UTF_8));
        }
        int code = conn.getResponseCode();
        conn.disconnect();
        return code;
    }

    @Test
    void aSecondDashboardMovesToTheNextFreePort() throws IOException {
        AprilTagDashboardServer second = new AprilTagDashboardServer(backend);
        try {
            assertTrue(server.port() != second.port(),
                    "the first dashboard holds 8080, so the second must move: "
                            + server.port() + " vs " + second.port());
            assertEquals(AprilTagDashboardServer.PORT + 2, second.port(),
                    "8081 belongs to the detection ingest, so the next free port is 8082");
        } finally {
            second.close();
        }
    }

    @Test
    void rootServesTheDashboardPage() throws IOException {
        String html = get("/");

        assertTrue(html.contains("<title>HiveTag Dashboard</title>"), "the page is titled HiveTag");
        assertTrue(html.contains("<h1>HiveTag Dashboard</h1>"), "the header carries the same name");
        assertTrue(html.contains("data:image/png;base64,"),
                "the Hive Vision logo is embedded, so the robot needs no second request");
        assertTrue(html.contains("id=\"dets\""), "the detections table is on the page");
        assertTrue(html.contains("id=\"tagSizeIn\""), "the tunable tag-size slider is present");
        assertTrue(html.contains("id=\"fovDeg\""), "the tunable FOV slider is present");
        assertTrue(html.contains("honeycomb-canvas") || html.contains("initHoneycomb"),
                "the honeycomb background is initialised by the page");
    }

    @Test
    void directionReadoutsAreNotRendered() throws IOException {
        String html = get("/");

        assertTrue(!html.contains("d.bearing"), "bearing is not drawn per detection");
        assertTrue(!html.contains("d.elevation"), "elevation is not drawn per detection");
        assertTrue(!html.contains("pitch/roll/yaw"), "orientation is not drawn per detection");
        assertTrue(html.contains("id=\"count\""), "the header still counts detections");
    }

    @Test
    void thePageCarriesTheSourceSelectorAndTheLimelightControls() throws IOException {
        String html = get("/");

        assertTrue(html.contains("id=\"cameraMode\""), "the source selector is on the page");
        assertTrue(html.contains("id=\"llRows\""),
                "the Limelight-only rows exist so the selector can show and hide them");
        assertTrue(html.contains("id=\"limelightName\""),
                "the device name the hardwareMap will look up");
        assertTrue(html.contains("id=\"limelightHost\""),
                "the host field, for a camera that is not at limelight.local");
        assertTrue(html.contains("__llHost"), "the link out reads that host first");
        assertTrue(html.contains("id=\"limelightPollHz\""), "the Limelight's own poll rate");
        assertTrue(html.contains("id=\"llUi\""), "the link out to the Limelight's config page");
        assertTrue(html.contains("5801"), "the link promises the Limelight UI's port");
        assertFalse(html.contains("5051"), "the old, wrong port is gone");
        assertTrue(html.contains("id=\"llDev\""), "the device's active pipeline is shown");
        assertTrue(html.contains("id=\"llPush\""), "the push-tag-size button is on the page");
        assertTrue(html.contains("id=\"llSync\""), "so is the line its result is written to");
        assertTrue(html.contains("pushTagSize"),
                "the button posts the one-shot request the OpMode consumes");
        assertTrue(html.contains("id=\"tagNames\""),
                "the id -> name table the alliance rule reads in Limelight mode");
        assertTrue(html.contains("id=\"camOnly\"") && html.contains("id=\"detOnly\""),
                "the webcam-only controls are grouped so they can be switched off");
        assertTrue(html.contains("<details open id=\"detOnly\">"),
                "the detector box as a whole, so Limelight mode leaves no empty heading behind");
        assertTrue(html.contains("id=\"resolution\""),
                "the webcam controls remain, for the webcam source");
    }

    @Test
    void stateEndpointReturnsWhatTheBackendPublished() throws IOException {
        backend.state = "{\"camState\":\"STREAMING\",\"detections\":[{\"id\":1}],"
                + "\"fps\":31.5,\"loopMs\":2}";

        String body = get("/api/state");
        assertTrue(body.contains("\"camState\":\"STREAMING\""), body);
        assertTrue(body.contains("\"id\":1"), body);
    }

    @Test
    void configPostReachesTheBackend() throws IOException {
        int code = post("/api/config", "{\"alliance\":\"BLUE\",\"decimation\":2}");

        assertEquals(200, code, "the dashboard needs an ack before it moves on");
        assertNotNull(backend.lastConfig, "the body must reach the OpMode's Backend");
        assertTrue(backend.lastConfig.contains("\"alliance\":\"BLUE\""), backend.lastConfig);
        assertTrue(backend.lastConfig.contains("\"decimation\":2"), backend.lastConfig);
    }

    @Test
    void aMalformedConfigPostIsRejectedOrIgnoredButNeverKillsTheServer() throws IOException {
        int code = post("/api/config", "this is not json");

        assertTrue(code == 400 || code == 200, "got " + code);
        // The server must still be serving after a bad body.
        assertTrue(get("/api/state").length() > 0);
    }

    /** Reads up to the blank line that ends a set of HTTP / MJPEG part headers. */
    private static String readHeaders(InputStream in) throws IOException {
        StringBuilder sb = new StringBuilder(256);
        int c;
        while ((c = in.read()) != -1) {
            sb.append((char) c);
            if (sb.length() >= 4 && sb.indexOf("\r\n\r\n", sb.length() - 4) >= 0) {
                break;
            }
        }
        return sb.toString();
    }

    private static byte[] readExact(InputStream in, int len) throws IOException {
        byte[] out = new byte[len];
        int off = 0;
        while (off < len) {
            int n = in.read(out, off, len - off);
            if (n < 0) {
                break;
            }
            off += n;
        }
        return java.util.Arrays.copyOf(out, off);
    }

    @Test
    void liveViewSaysThereAreNoFramesYet() throws IOException {
        HttpURLConnection conn = connect("GET", "/stream.mjpg");
        int code = conn.getResponseCode();
        conn.disconnect();

        assertEquals(503, code,
                "before the camera sends a frame the page must be told to retry, not hung");
        assertTrue(server.wantsFrames(),
                "that retry must wake the encoder, or the first frame is never taken");
    }

    @Test
    void liveViewServesTheNewestFrameAsMjpeg() throws IOException {
        // Not a real JPEG on purpose: the endpoint must hand through exactly what it is given.
        byte[] jpeg = {(byte) 0xFF, (byte) 0xD8, 1, 2, 3, (byte) 0xFF, (byte) 0xD9};
        server.publishFrame(jpeg);

        try (Socket socket = new Socket("127.0.0.1", AprilTagDashboardServer.PORT)) {
            socket.setSoTimeout(3000);
            OutputStream out = socket.getOutputStream();
            out.write("GET /stream.mjpg HTTP/1.1\r\nHost: 127.0.0.1\r\n\r\n"
                    .getBytes(StandardCharsets.US_ASCII));
            out.flush();
            InputStream in = socket.getInputStream();

            String response = readHeaders(in);
            assertTrue(response.startsWith("HTTP/1.1 200 OK"), response);
            assertTrue(response.contains("multipart/x-mixed-replace"), response);

            String part = readHeaders(in);
            assertTrue(part.contains("Content-Type: image/jpeg"), part);
            assertTrue(part.contains("Content-Length: " + jpeg.length), part);
            assertArrayEquals(jpeg, readExact(in, jpeg.length),
                    "the frame the OpMode published is the one the browser receives");
        }
    }

    /**
     * The Limelight streams for itself on :5802. Asked to relay such a feed, the dashboard has to
     * serve its frames on the same {@code /stream.mjpg} the webcam uses: the page keeps one URL,
     * and the video keeps working over the robot's wifi and through {@code adb reverse} alike.
     */
    @Test
    void liveViewRelaysACamerasOwnMjpegFeed() throws Exception {
        byte[] first = {(byte) 0xFF, (byte) 0xD8, 1, 2, 3, (byte) 0xFF, (byte) 0xD9};
        byte[] second = {(byte) 0xFF, (byte) 0xD8, 4, 5, (byte) 0xFF, (byte) 0xD9};
        byte[][] frames = {first, second};

        try (ServerSocket upstream = new ServerSocket(0)) {
            Thread camera = new Thread(() -> {
                try (Socket s = upstream.accept()) {
                    readHeaders(s.getInputStream()); // the relay's request
                    OutputStream out = s.getOutputStream();
                    out.write(("HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace;"
                            + " boundary=boundarydonotcross\r\n\r\n")
                            .getBytes(StandardCharsets.US_ASCII));
                    for (byte[] frame : frames) {
                        out.write(("--boundarydonotcross\r\nContent-Type: image/jpeg\r\n"
                                + "Content-Length: " + frame.length + "\r\n\r\n")
                                .getBytes(StandardCharsets.US_ASCII));
                        out.write(frame);
                        out.write("\r\n".getBytes(StandardCharsets.US_ASCII));
                        out.flush();
                    }
                    Thread.sleep(3000); // hold the feed open while the test reads a frame
                } catch (Exception ignored) {
                    // the server closed first: the test is over
                }
            }, "fake-limelight");
            camera.setDaemon(true);
            camera.start();

            server.setRelayUrl("http://127.0.0.1:" + upstream.getLocalPort() + "/stream.mjpg");

            byte[] received = null;
            long deadline = System.currentTimeMillis() + 5000;
            while (received == null && System.currentTimeMillis() < deadline) {
                try (Socket socket = new Socket("127.0.0.1", AprilTagDashboardServer.PORT)) {
                    socket.setSoTimeout(2000);
                    OutputStream out = socket.getOutputStream();
                    out.write("GET /stream.mjpg HTTP/1.1\r\nHost: 127.0.0.1\r\n\r\n"
                            .getBytes(StandardCharsets.US_ASCII));
                    out.flush();
                    InputStream in = socket.getInputStream();
                    if (readHeaders(in).startsWith("HTTP/1.1 200")) {
                        String part = readHeaders(in);
                        int length = Integer.parseInt(
                                part.replaceAll("(?s).*Content-Length: (\\d+).*", "$1"));
                        received = readExact(in, length);
                    }
                } catch (IOException e) {
                    // Nothing relayed yet: the page's retry, in miniature.
                }
                if (received == null) {
                    Thread.sleep(200);
                }
            }

            assertNotNull(received, "no frame was relayed from the camera's own stream");
            assertTrue(java.util.Arrays.equals(received, first)
                            || java.util.Arrays.equals(received, second),
                    "the browser receives a frame the camera sent, byte for byte");
            assertTrue(server.wantsFrames(), "the fresh request must keep the relay connected");
        }
    }

    @Test
    void closeReleasesThePort() throws IOException, InterruptedException {
        get("/api/state");
        server.close();
        server = null;

        boolean freed = false;
        for (int i = 0; i < 20; i++) {
            if (portFree(AprilTagDashboardServer.PORT)) {
                freed = true;
                break;
            }
            Thread.sleep(50);
        }
        assertTrue(freed,
                "stop() must hand 8080 back, or the next run cannot start");
    }
}
