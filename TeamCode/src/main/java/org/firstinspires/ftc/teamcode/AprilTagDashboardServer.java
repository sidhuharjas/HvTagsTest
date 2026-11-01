package org.firstinspires.ftc.teamcode;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.Enumeration;
import java.util.Locale;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Tiny localhost dashboard server that runs ON the Robot Controller / Control Hub.
 *
 *   GET  /              the tuning page (embedded, no internet needed)
 *   GET  /api/state     JSON snapshot of camera + detections + config
 *   POST /api/config    JSON with the tunables that changed; applied on the OpMode loop
 *
 * From the pit laptop:  http://<robot-ip>:8080
 * Or over USB:          adb reverse tcp:8080 tcp:8080  then  http://localhost:8080
 *
 * Implemented on plain sockets (not com.sun.net.httpserver) so the same class
 * compiles and runs on the Android FTC runtime, the desktop simulator, and the JVM.
 */
public class AprilTagDashboardServer {

    public interface Backend {
        /** Latest state JSON, built on the OpMode loop thread. */
        String snapshotJson();

        /** Partial config JSON from the dashboard; must be thread-safe. */
        void applyConfig(String jsonBody);
    }

    public static final int PORT = 8080;
    /** Where the dashboard moves when {@link #PORT} is taken - on a Control Hub it always is:
     *  the Robot Controller app serves OnBot Java and Blocks on 192.168.43.1:8080. */
    private static final int[] ALT_PORTS = {8082, 8083, 8084, 8085, 8086, 8087, 8088, 8089};

    private final Backend backend;
    private final ServerSocket serverSocket;
    private final int port;
    private final ExecutorService pool;
    private volatile boolean running = true;

    /** Newest webcam frame as JPEG, published by the OpMode's frame grabber. */
    private volatile byte[] latestJpeg;
    private volatile long latestJpegSeq;
    private volatile long latestJpegAtMs;
    /** Last time the page asked for video; keeps the first frame coming after a 503 retry. */
    private volatile long lastStreamRequestMs;
    /** Each viewer holds a thread and a socket, so cap them. */
    private final java.util.concurrent.atomic.AtomicInteger streamViewers =
            new java.util.concurrent.atomic.AtomicInteger();
    private static final int MAX_STREAM_VIEWERS = 4;

    public AprilTagDashboardServer(Backend backend) throws IOException {
        this.backend = backend;
        this.serverSocket = bind();
        this.port = serverSocket.getLocalPort();
        this.pool = Executors.newCachedThreadPool();
        pool.execute(new Runnable() {
            @Override
            public void run() {
                while (running) {
                    try {
                        final Socket s = serverSocket.accept();
                        pool.execute(new Runnable() {
                            @Override
                            public void run() {
                                handle(s);
                            }
                        });
                    } catch (IOException e) {
                        if (!running) {
                            return; // closed on purpose
                        }
                    }
                }
            }
        });
    }

    /** The port the dashboard actually ended up on; not {@link #PORT} when 8080 was busy. */
    public int port() {
        return port;
    }

    /**
     * True while a browser is watching, so encoding can be skipped otherwise.
     *
     * <p>A viewer only exists after the first frame, so a recent request counts too: without
     * that, the page's 503 retry would never be answered - nothing would ever encode the frame
     * that turns the retry into a stream.
     */
    public boolean wantsFrames() {
        return streamViewers.get() > 0
                || System.currentTimeMillis() - lastStreamRequestMs < 10_000;
    }

    /**
     * Publishes the newest webcam frame for {@code GET /stream.mjpg}. Called from the camera
     * thread, once per encoded frame, only while {@link #wantsFrames()} says a page is watching.
     */
    public void publishFrame(byte[] jpeg) {
        if (jpeg == null || jpeg.length == 0) {
            return;
        }
        latestJpeg = jpeg;
        latestJpegAtMs = System.currentTimeMillis();
        latestJpegSeq++;
    }

    /** Binds the first free port, so a Control Hub - where the RC app already owns 8080 - still
     *  gets a dashboard, just at 8082 or later. */
    private static ServerSocket bind() throws IOException {
        IOException last = null;
        for (int i = -1; i < ALT_PORTS.length; i++) {
            int candidate = i < 0 ? PORT : ALT_PORTS[i];
            ServerSocket socket = new ServerSocket();
            socket.setReuseAddress(true);
            try {
                socket.bind(new InetSocketAddress(candidate));
                return socket;
            } catch (IOException e) {
                last = e;
                try {
                    socket.close();
                } catch (IOException ignored) {
                    // nothing was bound, nothing to release
                }
            }
        }
        throw new IOException("no free port for the dashboard (" + PORT + ", then "
                + ALT_PORTS[0] + "-" + ALT_PORTS[ALT_PORTS.length - 1] + " are all in use)", last);
    }

    private void handle(Socket sock) {
        try (Socket s = sock;
             InputStream in = s.getInputStream();
             OutputStream out = s.getOutputStream()) {
            String requestLine = readLine(in);
            if (requestLine == null) {
                return;
            }
            String[] parts = requestLine.split(" ");
            if (parts.length < 2) {
                return;
            }
            String method = parts[0];
            String path = parts[1].split("\\?")[0];

            int contentLength = 0;
            String line;
            while ((line = readLine(in)) != null && !line.isEmpty()) {
                if (line.toLowerCase(Locale.US).startsWith("content-length:")) {
                    contentLength = Integer.parseInt(line.substring(15).trim());
                }
            }
            String body = contentLength > 0 ? readBody(in, contentLength) : "";

            if ("GET".equals(method) && "/stream.mjpg".equals(path)) {
                serveStream(out); // long-lived: no Content-Length, the connection frames it
                return;
            }

            int code;
            String contentType;
            byte[] bytes;
            if ("GET".equals(method) && ("/".equals(path) || "/index.html".equals(path))) {
                code = 200;
                contentType = "text/html; charset=utf-8";
                bytes = HTML.getBytes(StandardCharsets.UTF_8);
            } else if ("GET".equals(method) && "/api/state".equals(path)) {
                code = 200;
                contentType = "application/json; charset=utf-8";
                bytes = backend.snapshotJson().getBytes(StandardCharsets.UTF_8);
            } else if ("POST".equals(method) && "/api/config".equals(path)) {
                backend.applyConfig(body);
                code = 200;
                contentType = "application/json; charset=utf-8";
                bytes = "{\"ok\":true}".getBytes(StandardCharsets.UTF_8);
            } else {
                code = 404;
                contentType = "text/plain; charset=utf-8";
                bytes = ("not found: " + path).getBytes(StandardCharsets.UTF_8);
            }
            String status = code == 200 ? "OK" : code == 404 ? "Not Found" : "Error";
            String headers = "HTTP/1.1 " + code + " " + status + "\r\n"
                    + "Content-Type: " + contentType + "\r\n"
                    + "Content-Length: " + bytes.length + "\r\n"
                    + "Cache-Control: no-store\r\n"
                    + "Connection: close\r\n\r\n";
            out.write(headers.getBytes(StandardCharsets.UTF_8));
            out.write(bytes);
            out.flush();
        } catch (Exception ignored) {
            // one bad client must not kill the server
        }
    }

    /**
     * The webcam as MJPEG, from the dashboard's own port: the page can be opened over wifi or
     * through {@code adb reverse}, and the video comes from the same place as the JSON.
     *
     * <p>503 means no frame yet (the camera is still starting), so the page retries; once frames
     * flow this connection stays open and each new JPEG replaces the last. A browser that goes
     * away shows up as an IOException on write, and a camera that stops for 5 s closes the
     * stream so the page reconnects when it comes back.
     */
    private void serveStream(OutputStream out) throws IOException {
        lastStreamRequestMs = System.currentTimeMillis();
        if (latestJpeg == null || System.currentTimeMillis() - latestJpegAtMs > 3000) {
            writeText(out, 503, "no camera frames yet");
            return;
        }
        if (streamViewers.incrementAndGet() > MAX_STREAM_VIEWERS) {
            streamViewers.decrementAndGet();
            writeText(out, 503, "too many live view connections");
            return;
        }
        try {
            out.write(("HTTP/1.1 200 OK\r\n"
                    + "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
                    + "Cache-Control: no-store, no-cache, must-revalidate\r\n"
                    + "Pragma: no-cache\r\n"
                    + "Connection: close\r\n\r\n").getBytes(StandardCharsets.US_ASCII));
            out.flush();
            long sent = -1;
            while (running) {
                if (System.currentTimeMillis() - latestJpegAtMs > 5000) {
                    break; // the camera stopped; ending the response lets the page reconnect
                }
                long seq = latestJpegSeq;
                byte[] jpeg = latestJpeg;
                if (jpeg != null && seq != sent) {
                    out.write(("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                            + jpeg.length + "\r\n\r\n").getBytes(StandardCharsets.US_ASCII));
                    out.write(jpeg);
                    out.write(new byte[]{'\r', '\n'});
                    out.flush();
                    sent = seq;
                }
                Thread.sleep(15);
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } finally {
            streamViewers.decrementAndGet();
        }
    }

    private static void writeText(OutputStream out, int code, String text) throws IOException {
        byte[] body = text.getBytes(StandardCharsets.UTF_8);
        out.write(("HTTP/1.1 " + code + " Service Unavailable\r\n"
                + "Content-Type: text/plain; charset=utf-8\r\n"
                + "Content-Length: " + body.length + "\r\n"
                + "Cache-Control: no-store\r\n"
                + "Connection: close\r\n\r\n").getBytes(StandardCharsets.US_ASCII));
        out.write(body);
        out.flush();
    }

    public void close() {
        running = false;
        try {
            serverSocket.close();
        } catch (IOException ignored) {
        }
        pool.shutdownNow();
    }

    /** Best-effort IPv4 of the robot's wifi interface, for the "open this URL" hint. */
    public static String firstLanIp() {
        try {
            Enumeration<NetworkInterface> ifs = NetworkInterface.getNetworkInterfaces();
            while (ifs != null && ifs.hasMoreElements()) {
                NetworkInterface nif = ifs.nextElement();
                Enumeration<InetAddress> addrs = nif.getInetAddresses();
                while (addrs.hasMoreElements()) {
                    InetAddress a = addrs.nextElement();
                    if (a instanceof Inet4Address && a.isSiteLocalAddress() && !a.isLoopbackAddress()) {
                        return a.getHostAddress();
                    }
                }
            }
        } catch (Exception ignored) {
        }
        return "robot-ip";
    }

    private static String readLine(InputStream in) throws IOException {
        ByteArrayOutputStream buf = new ByteArrayOutputStream(128);
        int c;
        while ((c = in.read()) != -1) {
            if (c == '\n') {
                break;
            }
            if (c != '\r') {
                buf.write(c);
            }
        }
        if (c == -1 && buf.size() == 0) {
            return null;
        }
        return new String(buf.toByteArray(), StandardCharsets.UTF_8);
    }

    private static String readBody(InputStream in, int len) throws IOException {
        byte[] b = new byte[len];
        int off = 0;
        while (off < len) {
            int n = in.read(b, off, len - off);
            if (n < 0) {
                break;
            }
            off += n;
        }
        return new String(b, 0, off, StandardCharsets.UTF_8);
    }

    private static final String HTML = join(new String[]{
            "<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\">",
            "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">",
            "<title>HiveTag Dashboard</title>",
            "<script>(function(){var t=localStorage.getItem('hv-theme')||'dark';",
            "document.documentElement.setAttribute('data-theme',t);",
            "window.toggleTheme=function(){var n=document.documentElement.getAttribute('data-theme')==='dark'?'light':'dark';",
            "document.documentElement.setAttribute('data-theme',n);localStorage.setItem('hv-theme',n);",
            "var b=document.getElementById('themeBtn');if(b)b.textContent=n==='dark'?'light mode':'dark mode';",
            "var c=document.getElementById('honeycomb-canvas');",
            "if(!c&&window.initHoneycomb)window.initHoneycomb();};})();</script>",
            "<style>",
            /* Hive Vision "lunar-light": black page, fixed gradient, white glow.
               Dark is the default, matching the docs; the header button switches themes. */
            ":root[data-theme=dark]{--bg:#000000;--bg-image:linear-gradient(to bottom,#16161a 0%,#0a0a0c 35%,#000000 100%);",
            "--surface:rgba(255,255,255,.02);--bg-soft:rgba(255,255,255,.05);--bg-soft2:rgba(255,255,255,.09);",
            "--fg:#eceef1;--dim:#8b9096;--line:rgba(255,255,255,.10);--line-strong:rgba(255,255,255,.24);",
            "--ok:#5ad17f;--ok-soft:rgba(90,209,127,.12);--bad:#eceef1;--bad-soft:rgba(255,255,255,.10);",
            "--glow:0 10px 30px rgba(255,255,255,.08),0 1px 3px rgba(255,255,255,.15);",
            "--glow-hi:0 15px 40px rgba(255,255,255,.12),0 1px 5px rgba(255,255,255,.25)}",
            ":root[data-theme=light]{--bg:#ffffff;--bg-image:none;--surface:#ffffff;",
            "--bg-soft:rgba(0,0,0,.04);--bg-soft2:rgba(0,0,0,.07);",
            "--fg:#17181a;--dim:#6a6e77;--line:rgba(0,0,0,.10);--line-strong:rgba(0,0,0,.22);",
            "--ok:#1a9e4d;--ok-soft:rgba(26,158,77,.10);--bad:#17181a;--bad-soft:rgba(0,0,0,.07);",
            "--glow:0 1px 2px rgba(0,0,0,.06),0 6px 18px rgba(0,0,0,.05);",
            "--glow-hi:0 2px 4px rgba(0,0,0,.08),0 12px 28px rgba(0,0,0,.10)}",
            "*{box-sizing:border-box}",
            "html{background:var(--bg)}",
            "body{margin:0;color:var(--fg);background-image:var(--bg-image);background-attachment:fixed;",
            "font:14px/1.5 system-ui,-apple-system,\"Segoe UI\",Roboto,Helvetica,Arial,sans-serif}",
            "header{position:sticky;top:0;z-index:5;display:flex;gap:8px;align-items:center;flex-wrap:wrap;padding:14px 20px;",
            "background:linear-gradient(to bottom,var(--bg),transparent);backdrop-filter:blur(6px)}",
            "h1{font-size:15px;margin:0 14px 0 4px;font-weight:600}",
            ".logo{width:22px;height:22px;flex:0 0 auto;display:block}",
            ":root[data-theme=light] .logo{filter:invert(1)}",
            ".chip{border-radius:8px;padding:4px 10px;color:var(--dim);font-size:12px;background:var(--bg-soft)}",
            ".chip b{color:var(--fg);font-weight:600}",
            "#themeBtn{margin-left:auto;background:var(--bg-soft);color:var(--dim);border:1px solid var(--line);",
            "border-radius:8px;padding:5px 12px;font:inherit;font-size:12px;cursor:pointer}",
            "#themeBtn:hover{color:var(--fg);border-color:var(--line-strong)}",
            "main{position:relative;z-index:1;display:grid;grid-template-columns:1fr 372px;gap:22px;padding:22px;max-width:1400px;margin:0 auto}",
            "@media(max-width:1080px){main{grid-template-columns:1fr}}",
            ".card{background:var(--surface);border:1px solid var(--line);border-radius:14px;padding:16px 18px;margin-bottom:18px;",
            "box-shadow:var(--glow);transition:box-shadow .2s ease}",
            ".card:hover{box-shadow:var(--glow-hi)}",
            ".card h2{margin:0 0 12px;font-size:11px;letter-spacing:.1em;text-transform:uppercase;font-weight:600;color:var(--dim)}",
            ".row{display:flex;justify-content:space-between;align-items:center;gap:10px;margin:9px 0}",
            ".row .val{color:var(--dim);font-variant-numeric:tabular-nums;min-width:52px;text-align:right;font-size:12px;",
            "font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}",
            "input[type=range]{width:158px;accent-color:currentColor}",
            "select,input[type=text]{background:var(--bg-soft);color:var(--fg);border:1px solid var(--line-strong);border-radius:8px;padding:5px 8px;font:inherit}",
            "input[type=checkbox]{accent-color:currentColor}",
            ".seg{display:flex;border:1px solid var(--line-strong);border-radius:8px;overflow:hidden;background:var(--bg-soft)}",
            ".seg button{background:transparent;color:var(--dim);border:0;padding:5px 14px;cursor:pointer;font:inherit}",
            ".seg button.on{background:var(--fg);color:var(--bg)}",
            ".btn{background:var(--bg-soft);color:var(--fg);border:1px solid var(--line-strong);border-radius:8px;padding:7px 14px;cursor:pointer;font:inherit;font-weight:500}",
            ".btn:hover{background:var(--bg-soft2)}",
            ".btns{display:flex;gap:8px;justify-content:space-between}",
            "details{border-top:1px solid var(--line);padding-top:2px}",
            "details:first-of-type{border-top:0;padding-top:0}",
            "summary{cursor:pointer;list-style:none;padding:10px 0 6px;font-size:11px;letter-spacing:.1em;",
            "text-transform:uppercase;font-weight:600;color:var(--dim)}",
            "summary::-webkit-details-marker{display:none}",
            "summary:hover,summary[open]{color:var(--fg)}",
            "#targetCard{border-color:var(--line-strong);background:var(--bg-soft)}",
            "#targetCard h2{margin-bottom:8px}",
            "#targetCard .big{font-size:15px;font-weight:600;color:var(--fg)}",
            "#targetCard .dist{font:600 18px/1.4 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;",
            "color:var(--dim);font-variant-numeric:tabular-nums}",
            "#targetCard .sub{color:var(--dim);font-size:12px;margin-top:3px;font-variant-numeric:tabular-nums;",
            "font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}",
            ".hero{display:flex;justify-content:space-between;align-items:flex-end;gap:16px}",
            ".badge{font-size:10px;letter-spacing:.06em;text-transform:uppercase;border-radius:5px;padding:2px 7px;",
            "background:var(--bg-soft2);color:var(--dim);white-space:nowrap}",
            ".badge.cluster{background:var(--ok-soft);color:var(--ok)}",
            ".badge.scr{background:var(--ok);color:#fff}",
            ".tscroll{overflow-x:auto}",
            ".dtable{width:100%;border-collapse:collapse}",
            ".dtable th{font-size:10px;letter-spacing:.08em;text-transform:uppercase;color:var(--dim);font-weight:600;",
            "text-align:left;padding:0 8px 8px;border-bottom:1px solid var(--line);white-space:nowrap}",
            ".dtable td{padding:9px 8px;border-bottom:1px solid var(--line);font-size:13px;white-space:nowrap}",
            ".dtable tr:last-child td{border-bottom:0}",
            ".dtable tr.unsc td{opacity:.45}",
            ".dtable tr.scr td{background:var(--ok-soft)}",
            ".dtable .num{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;font-variant-numeric:tabular-nums;",
            "text-align:right;color:var(--dim)}",
            ".dtable .num b{color:var(--fg);font-weight:600}",
            "#raw{width:100%;height:96px;background:var(--bg-soft);color:var(--dim);border:1px solid var(--line);border-radius:10px;",
            "font:11px/1.4 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;resize:vertical;margin-top:8px}",
            ".hint{color:var(--dim);font-size:12px;margin-top:10px;line-height:1.5}",
            "#errCard{border-color:rgba(220,38,38,.55);background:rgba(220,38,38,.07)}",
            "#errCard[hidden]{display:none!important}",
            "#err{color:#e5484d;font:12px/1.5 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;",
            "word-break:break-word;white-space:pre-wrap}",
            ".hiveState{font:700 26px/1.2 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}",
            ".hiveState.up{color:var(--fg)}",
            ".hiveState.down{display:inline-block;padding:2px 12px;border-radius:8px;background:var(--fg);color:var(--bg)}",
            "#hiveCard.tipped{border-color:var(--fg);background:var(--bg-soft2)}",
            "#honeycomb-canvas{position:fixed;top:0;left:0;width:100vw;height:100vh;pointer-events:none;z-index:0;display:block}",
            ".main-frame{box-shadow:0 0 80px 10px rgba(255,255,255,0.05)}",
            "</style></head><body>",
            "<header><img class=\"logo\" src=\"data:image/png;base64,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\" alt=\"Hive Vision\">",
            "<h1>HiveTag Dashboard</h1>",
            "<span class=\"chip\">tags <b id=\"count\">0</b></span>",
            "<span class=\"chip\">fps <b id=\"fps\">-</b></span>",
            "<span class=\"chip\">loop <b id=\"loop\">-</b></span>",
            "<span class=\"chip\">cam <b id=\"cam\">-</b></span>",
            "<button id=\"themeBtn\" onclick=\"toggleTheme()\">light mode</button>",
            "</header><main><div>",
            // Only shown when a loop threw: the OpMode keeps running, so the reason matters.
            "<div class=\"card\" id=\"errCard\" hidden><h2>OpMode error</h2>",
            "<div id=\"err\">-</div>",
            "<div class=\"hint\">The OpMode is still running - this is what the last loop hit.</div></div>",

            "<div class=\"card\" id=\"targetCard\"><h2>Best scorable target</h2>",
            "<div class=\"hero\"><div><div class=\"big\" id=\"tName\">none in view</div>",
            "<div class=\"sub\" id=\"tSub\">-</div></div>",
            "<div class=\"dist\" id=\"tDist\"></div></div></div>",

            "<div class=\"card\"><h2>Live view</h2>",
            "<img id=\"camStream\" alt=\"camera stream\"",
            " style=\"width:100%;border-radius:8px;background:var(--bg-soft2);min-height:160px;object-fit:contain;display:none\">",
            "<div id=\"camStreamOff\" class=\"hint\">waiting for the camera stream...</div></div>",

            "<div class=\"card\"><h2>Detections</h2><div class=\"tscroll\"><table class=\"dtable\"><thead><tr>",
            "<th>target</th><th>type</th><th style=\"text-align:right\">distance</th>",
            "<th style=\"text-align:right\">decode</th>",
            "</tr></thead><tbody id=\"dets\"></tbody></table></div></div>",

            "</div><div>",

            "<div class=\"card\" id=\"hiveCard\"><h2>Hive cell</h2>",
            "<div class=\"hiveState\" id=\"hiveState\">-</div>",
            "<div class=\"hint\" id=\"hiveSub\">-</div>",
            "<div class=\"row\"><label>baseline pitch <span class=\"val\" id=\"hbVal\">0.0</span></label>",
            "<input type=\"range\" id=\"hiveBaselinePitchDeg\" min=\"-90\" max=\"90\" step=\"0.5\" value=\"0\"></div>",
            "<div class=\"row\"><label>tipped threshold <span class=\"val\" id=\"htVal\">25.0</span></label>",
            "<input type=\"range\" id=\"hiveTippedThresholdDeg\" min=\"1\" max=\"90\" step=\"1\" value=\"25\"></div>",
            "<div class=\"row\"><button class=\"btn\" id=\"hiveCalBtn\">calibrate baseline now</button></div></div>",

            "<div class=\"card\"><h2>Controls</h2>",
            "<details open><summary>camera</summary>",
            "<div class=\"row\"><label>source</label><select id=\"cameraMode\">",
            "<option value=\"WEBCAM\">USB webcam</option><option value=\"LIMELIGHT\">Limelight 3A</option>",
            "</select></div>",
            "<div id=\"llRows\" hidden>",
            "<div class=\"row\"><label>Limelight device</label>",
            "<input type=\"text\" id=\"limelightName\" size=\"12\"></div>",
            "<div class=\"row\"><label>host</label>",
            "<input type=\"text\" id=\"limelightHost\" size=\"16\" placeholder=\"auto (limelight.local)\"></div>",
            "<div class=\"row\"><label>poll rate <span class=\"val\" id=\"llHzVal\">20</span> Hz</label>",
            "<input type=\"range\" id=\"limelightPollHz\" min=\"1\" max=\"120\" step=\"1\" value=\"20\"></div>",
            "<div class=\"row\"><label>active pipeline</label><span class=\"val\" id=\"llDev\">-</span></div>",
            "<div class=\"row\"><button class=\"btn\" id=\"llPush\">push tag size &rarr; Limelight</button></div>",
            "<div class=\"hint\" id=\"llSync\">nothing pushed yet</div>",
            "<div class=\"row\"><a id=\"llUi\" target=\"_blank\" rel=\"noopener\">open Limelight UI :5801</a></div>",
            "<div class=\"hint\">exposure, LED and the tag family live in the Limelight's own "
            + "pipeline - configure them there; this page keeps the robot-side rules.</div>",
            "</div>",
            "<div id=\"camOnly\">",
            "<div class=\"row\"><label>resolution</label><select id=\"resolution\">",
            "<option value=\"0\">640 x 480</option><option value=\"1\">1280 x 720</option></select></div>",
            "<div class=\"row\"><label>camera name</label><input type=\"text\" id=\"cameraName\" size=\"12\"></div>",
            "<div class=\"row\"><label>tag size <span class=\"val\" id=\"tsVal\">5.0</span> in</label>",
            "<input type=\"range\" id=\"tagSizeIn\" min=\"0.5\" max=\"24\" step=\"0.1\" value=\"5\"></div>",
            "<div class=\"row\"><label>camera FOV <span class=\"val\" id=\"fovVal\">65</span> deg</label>",
            "<input type=\"range\" id=\"fovDeg\" min=\"30\" max=\"140\" step=\"1\" value=\"65\"></div>",
            "<div class=\"row\"><label>auto exposure</label><input type=\"checkbox\" id=\"aeOn\" checked></div>",
            "<div class=\"row\"><label>exposure <span class=\"val\" id=\"expVal\">auto</span></label>",
            "<input type=\"range\" id=\"exposureMs\" min=\"1\" max=\"1000\" step=\"5\" value=\"50\"></div>",
            "<div class=\"row\"><label>manual gain</label><input type=\"checkbox\" id=\"gainOn\"></div>",
            "<div class=\"row\"><label>gain <span class=\"val\" id=\"gainVal\">-</span></label>",
            "<input type=\"range\" id=\"gain\" min=\"0\" max=\"255\" step=\"1\" value=\"20\"></div>",
            "<div class=\"row\"><label>DS live view</label><input type=\"checkbox\" id=\"liveView\" checked></div>",
            "</div>", // #camOnly
            "</details>",

            "<details open id=\"detOnly\"><summary>detector</summary>",
            "<div class=\"row\"><label>decimation <span class=\"val\" id=\"decVal\">1.0</span></label>",
            "<input type=\"range\" id=\"decimation\" min=\"1\" max=\"4\" step=\"0.5\" value=\"1\"></div>",
            "<div class=\"row\"><label>pose solver</label><select id=\"poseSolver\">",
            "<option selected>OPENCV_ITERATIVE</option><option>OPENCV_IPPE_SQUARE</option><option>OPENCV_IPPE</option>",
            "<option>OPENCV_SOLVEPNP_EPNP</option><option>OPENCV_SQPNP</option><option>APRILTAG_BUILTIN</option></select></div>",
            "</details>", // #detOnly: the whole box hides in Limelight mode - its settings are not ll settings

            "<details><summary>targeting</summary>",
            "<div class=\"row\"><label>alliance</label><div class=\"seg\" id=\"alliance\">",
            "<button data-v=\"RED\">RED</button><button data-v=\"BLUE\">BLUE</button><button data-v=\"ANY\">ANY</button>",
            "</div></div>",
            "<div class=\"row\"><label>max |roll| <span class=\"val\" id=\"rollVal\">90</span></label>",
            "<input type=\"range\" id=\"maxRollDeg\" min=\"5\" max=\"180\" step=\"5\" value=\"90\"></div>",
            "<div class=\"row\"><label>min cluster % <span class=\"val\" id=\"cluVal\">0</span></label>",
            "<input type=\"range\" id=\"minClusterPercent\" min=\"0\" max=\"100\" step=\"5\" value=\"0\"></div>",
            "<div class=\"row\"><label>min decision margin <span class=\"val\" id=\"marVal\">0</span></label>",
            "<input type=\"range\" id=\"minDecisionMargin\" min=\"0\" max=\"200\" step=\"5\" value=\"0\"></div>",
            "<div class=\"row\"><label>show unscorable</label><input type=\"checkbox\" id=\"showUnscorable\" checked></div>",
            "<div class=\"row\"><label>tag names</label>",
            "<input type=\"text\" id=\"tagNames\" size=\"14\" placeholder=\"{&quot;1&quot;:&quot;R_HIVE&quot;}\"></div>",
            "<div class=\"hint\">id -&gt; name, used in Limelight mode (webcam names come from the "
            + "SDK tag library). This is what the alliance rule reads.</div>",
            "</details></div>",

            "<div class=\"card\"><details><summary>maintenance</summary>",
            "<div class=\"btns\"><button class=\"btn\" id=\"resetBtn\">reset defaults</button>",
            "<button class=\"btn\" id=\"copyBtn\">copy state JSON</button></div>",
            "<textarea id=\"raw\" readonly></textarea></details></div>",

            "</div>",
            "</main><script>",
            "const $=id=>document.getElementById(id);",
            "const post=p=>fetch('/api/config',{method:'POST',body:JSON.stringify(p)}).catch(()=>{});",
            "const fmt=(v,d=1)=>v==null?'-':Number(v).toFixed(d);",
            "const esc=s=>String(s).replace(/[&<>\"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;',\"'\":'&#39;'}[c]));",
            // The video comes from the same server as this page: /stream.mjpg on the robot,
            // the Python preview on the desktop sim. Only touch src when the URL changes, or
            // the MJPEG connection would restart on every poll.
            "let streamUrl=null,streamRetry=null;",
            "function setStream(url){const img=$('camStream'),off=$('camStreamOff');",
            " if(url===streamUrl)return;streamUrl=url;",
            " if(streamRetry){clearTimeout(streamRetry);streamRetry=null;}",
            " if(!url){img.style.display='none';img.removeAttribute('src');off.hidden=false;",
            "  off.textContent='no live camera stream for this source';return;}",
            " off.hidden=true;img.style.display='block';img.src=url;}",
            "$('camStream').addEventListener('error',()=>{const img=$('camStream'),off=$('camStreamOff');",
            " img.style.display='none';img.removeAttribute('src');off.hidden=false;",
            " off.textContent='camera stream not ready yet - retrying...';",
            " if(streamRetry)clearTimeout(streamRetry);",
            " streamRetry=setTimeout(()=>{streamRetry=null;const u=streamUrl;streamUrl=null;setStream(u);},5000);});",
            "$('camStream').addEventListener('load',()=>{$('camStreamOff').hidden=true;",
            " $('camStream').style.display='block';});",
            "function wireRange(id,key,label,dec){const el=$(id),lab=$(label);",
            " el.addEventListener('change',()=>{lab.textContent=fmt(el.value,dec);post({[key]:parseFloat(el.value)});});}",
            "wireRange('decimation','decimation','decVal',1);",
            "wireRange('maxRollDeg','maxRollDeg','rollVal',0);",
            "wireRange('minClusterPercent','minClusterPercent','cluVal',0);",
            "wireRange('minDecisionMargin','minDecisionMargin','marVal',0);",
            "wireRange('hiveBaselinePitchDeg','hiveBaselinePitchDeg','hbVal',1);",
            "wireRange('hiveTippedThresholdDeg','hiveTippedThresholdDeg','htVal',1);",
            "wireRange('tagSizeIn','tagSizeIn','tsVal',1);",
            "wireRange('fovDeg','fovDeg','fovVal',0);",
            "$('hiveCalBtn').addEventListener('click',()=>{if(lastPitch!=null)post({hiveBaselinePitchDeg:lastPitch});});",
            "$('exposureMs').addEventListener('change',()=>{if($('aeOn').checked)return;",
            " post({exposureUs:Math.round(1000*parseFloat($('exposureMs').value))});});",
            "$('aeOn').addEventListener('change',()=>{post({exposureUs:$('aeOn').checked?-1:Math.round(1000*parseFloat($('exposureMs').value))});});",
            "$('gain').addEventListener('change',()=>{if(!$('gainOn').checked)return;post({gain:parseInt($('gain').value)});});",
            "$('gainOn').addEventListener('change',()=>{post({gain:$('gainOn').checked?parseInt($('gain').value):-1});});",
            "$('resolution').addEventListener('change',()=>post({resolution:parseInt($('resolution').value)}));",
            "$('cameraName').addEventListener('change',()=>post({cameraName:$('cameraName').value}));",
            "function llHref(ip){const h=(window.__llHost||'').trim();",
            " if(h)return 'http://'+h+'/5801/';",
            " const m=ip&&String(ip).match(/\\d{1,3}(\\.\\d{1,3}){3}/);",
            " if(m)return 'http://'+m[0]+'/5801/';",
            " if(ip){const p=String(ip).split('/')[0].split(':')[0];",
            "  if(p&&p!=='sim')return 'http://'+p+'/5801/';}",
            " return 'http://limelight.local:5801/';}",
            "function applyMode(m){const ll=m==='LIMELIGHT';",
            " $('camOnly').hidden=ll;$('detOnly').hidden=ll;$('llRows').hidden=!ll;",
            " $('llUi').href=llHref(window.__llIp);}",
            "$('cameraMode').addEventListener('change',()=>{applyMode($('cameraMode').value);",
            " post({cameraMode:$('cameraMode').value});});",
            "$('limelightName').addEventListener('change',()=>post({limelightName:$('limelightName').value}));",
            "$('limelightHost').addEventListener('change',()=>post({limelightHost:$('limelightHost').value}));",
            "wireRange('limelightPollHz','limelightPollHz','llHzVal',0);",
            "$('llPush').addEventListener('click',()=>{window.__llSync=null;",
            " $('llSync').textContent='sending...';post({pushTagSize:true});});",
            "$('tagNames').addEventListener('change',()=>post({tagNames:$('tagNames').value}));",
            "$('liveView').addEventListener('change',()=>post({liveView:$('liveView').checked}));",
            "$('showUnscorable').addEventListener('change',()=>post({showUnscorable:$('showUnscorable').checked}));",
            "$('poseSolver').addEventListener('change',()=>post({poseSolver:$('poseSolver').value}));",
            "document.querySelectorAll('#alliance button').forEach(b=>b.addEventListener('click',()=>post({alliance:b.dataset.v})));",
            "$('resetBtn').addEventListener('click',()=>post({cameraName:'Webcam 1',resolution:0,decimation:1,exposureUs:-1,",
             " gain:-1,tagSizeIn:5,fovDeg:65,alliance:'RED',maxRollDeg:90,minClusterPercent:0,minDecisionMargin:0,poseSolver:'OPENCV_ITERATIVE',",
             " showUnscorable:true,liveView:false,cameraMode:'WEBCAM',limelightName:'limelight',",
             " limelightHost:'',limelightPollHz:20,tagNames:'{}'}));",
            "$('copyBtn').addEventListener('click',()=>{navigator.clipboard&&navigator.clipboard.writeText($('raw').value);});",
            "function setUnfocused(el,fn){if(el&&document.activeElement!==el)fn(el);}",
            "function sync(cfg){if(!cfg)return;",
            " setUnfocused($('resolution'),e=>e.value=cfg.resolution);",
            " setUnfocused($('cameraName'),e=>{if(e.value!==cfg.cameraName)e.value=cfg.cameraName;});",
            " setUnfocused($('decimation'),e=>{e.value=cfg.decimation;$('decVal').textContent=fmt(cfg.decimation,1);});",
            " setUnfocused($('maxRollDeg'),e=>{e.value=cfg.maxRollDeg;$('rollVal').textContent=fmt(cfg.maxRollDeg,0);});",
            " setUnfocused($('minClusterPercent'),e=>{e.value=cfg.minClusterPercent;$('cluVal').textContent=cfg.minClusterPercent;});",
            " setUnfocused($('minDecisionMargin'),e=>{e.value=cfg.minDecisionMargin;$('marVal').textContent=cfg.minDecisionMargin;});",
            " setUnfocused($('poseSolver'),e=>e.value=cfg.poseSolver);",
            " setUnfocused($('aeOn'),e=>{e.checked=cfg.exposureUs<0;$('expVal').textContent=e.checked?'auto':fmt(cfg.exposureUs/1000,0)+' ms';});",
            " setUnfocused($('exposureMs'),e=>{if(cfg.exposureUs>=0)e.value=cfg.exposureUs/1000;});",
            " setUnfocused($('gainOn'),e=>{e.checked=cfg.gain>=0;$('gainVal').textContent=e.checked?cfg.gain:'auto';});",
            " setUnfocused($('gain'),e=>{if(cfg.gain>=0)e.value=cfg.gain;});",
            " setUnfocused($('tagSizeIn'),e=>{e.value=cfg.tagSizeIn;$('tsVal').textContent=fmt(cfg.tagSizeIn,1);});",
            " setUnfocused($('fovDeg'),e=>{e.value=cfg.fovDeg;$('fovVal').textContent=fmt(cfg.fovDeg,0);});",             " setUnfocused($('liveView'),e=>e.checked=cfg.liveView);",
             " setUnfocused($('hiveBaselinePitchDeg'),e=>{e.value=cfg.hiveBaselinePitchDeg;$('hbVal').textContent=fmt(cfg.hiveBaselinePitchDeg,1);});",
             " setUnfocused($('hiveTippedThresholdDeg'),e=>{e.value=cfg.hiveTippedThresholdDeg;$('htVal').textContent=fmt(cfg.hiveTippedThresholdDeg,1);});",
            " setUnfocused($('showUnscorable'),e=>e.checked=cfg.showUnscorable);",
            " setUnfocused($('cameraMode'),e=>{e.value=cfg.cameraMode||'WEBCAM';applyMode(e.value);});",
            " setUnfocused($('limelightName'),e=>{const v=cfg.limelightName||'limelight';if(e.value!==v)e.value=v;});",
            " setUnfocused($('limelightHost'),e=>{const v=cfg.limelightHost||'';if(e.value!==v)e.value=v;});",
            " setUnfocused($('limelightPollHz'),e=>{e.value=cfg.limelightPollHz;$('llHzVal').textContent=cfg.limelightPollHz;});",
            " setUnfocused($('tagNames'),e=>{const v=cfg.tagNames||'{}';if(e.value!==v)e.value=v;});",
            " document.querySelectorAll('#alliance button').forEach(b=>b.classList.toggle('on',b.dataset.v===cfg.alliance));}",
            "function detRow(d){",
            " const kind=d.kind||'single';",
            " const title=d.name!=null?esc(d.name):(d.id!=null?'ID '+d.id:'unknown');",
            " const dec=kind==='cluster'?fmt(d.percent,0)+'% visible':'margin '+fmt(d.margin,0);",
            " return '<tr class=\"'+(d.scorable?'scr':'unsc')+'\">'",
            "  +'<td><b>'+title+'</b></td>'",
            "  +'<td><span class=\"badge '+kind+'\">'+kind+'</span>'",
            "  +(d.scorable?' <span class=\"badge scr\">scorable</span>':'')+'</td>'",
            "  +'<td class=\"num\"><b>'+(d.hasPose===false?'no pose':fmt(d.range))+'</b>'+(d.hasPose===false?'':' in')+'</td>'",
            "  +'<td class=\"num\">'+dec+'</td></tr>';}",
            "let lastPitch=null;",
            "async function poll(){",
            " let s;try{s=await (await fetch('/api/state')).json();}catch(e){$('cam').textContent='disconnected';return;}",
             " $('cam').textContent=s.camState||'-';$('fps').textContent=fmt(s.fps,1);",
             " window.__llHost=(s.config&&s.config.limelightHost)||'';window.__llIp=s.llIp||null;$('llUi').href=llHref(window.__llIp);",
             " if(s.llPipe)$('llDev').textContent=s.llPipe;",
             " if(s.llSync&&s.llSync!==window.__llSync){window.__llSync=s.llSync;$('llSync').textContent=s.llSync;}",
             " $('loop').textContent=fmt(s.loopMs,1)+' ms';$('count').textContent=(s.detections||[]).length;",
             " setStream(s.streamUrl||'');",
            " if(s.error){$('err').textContent=s.error;$('errCard').hidden=false;}else{$('errCard').hidden=true;}",
            " ",             " sync(s.config);",
             " const hv=s.hive||{};",
             " lastPitch=(hv.pitch==null?null:hv.pitch);",
             " $('hiveState').textContent=hv.isTipped?'TIPPED':'UPRIGHT';",
             " $('hiveState').className='hiveState '+(hv.isTipped?'down':'up');",
             " $('hiveCard').classList.toggle('tipped',!!hv.isTipped);",
             " $('hiveSub').textContent='pitch '+(hv.pitch==null?'-':fmt(hv.pitch))+' deg  baseline '+fmt(hv.baselinePitchDeg)+' threshold '+fmt(hv.thresholdDeg);",
            " if(s.target){$('tName').textContent=s.target.name||('ID '+s.target.id);",
            "  $('tDist').textContent=fmt(s.target.range)+' in';",
            "  $('tSub').textContent=(s.target.kind||'single')+'  ·  '",
            "   +(s.target.scorable?'scorable':'not scorable')+'  ·  '",
            "   +((s.target.kind||'single')==='cluster'?fmt(s.target.percent,0)+'% visible':'margin '+fmt(s.target.margin,0));",
            "  $('targetCard').classList.add('has');}else{$('tName').textContent='none scorable';",
            "  $('tDist').textContent='';$('tSub').textContent=s.targetWhy||'-';$('targetCard').classList.remove('has');}",
            " const ds=(s.detections||[]).filter(d=>s.config&&s.config.showUnscorable||d.scorable);",
            " $('dets').innerHTML=ds.length?ds.map(detRow).join('')",
            "  :'<tr><td colspan=\"4\" style=\"text-align:center;color:var(--dim);padding:24px\">No AprilTags in view</td></tr>';",
            " $('raw').value=JSON.stringify(s);}",
            "$('themeBtn').textContent=document.documentElement.getAttribute('data-theme')==='dark'?'light mode':'dark mode';",
            "function initHoneycomb(){",
            " if(document.getElementById('honeycomb-canvas'))return;",
            " var canvas=document.createElement('canvas');canvas.id='honeycomb-canvas';document.body.prepend(canvas);",
            " var ctx=canvas.getContext('2d');",
            " var width=canvas.width=window.innerWidth,height=canvas.height=window.innerHeight;",
            " var mouse={x:width/2,y:height/2,active:false};",
            " var hexRadius=15,hexHeight=hexRadius*2,hexWidth=Math.sqrt(3)*hexRadius,vertDist=hexHeight*0.75;",
            " window.addEventListener('mousemove',function(e){mouse.x=e.clientX;mouse.y=e.clientY;mouse.active=true;});",
            " window.addEventListener('mouseleave',function(){mouse.active=false;});",
            " window.addEventListener('resize',function(){width=canvas.width=window.innerWidth;height=canvas.height=window.innerHeight;});",
            " function drawHexagon(cx,cy,r){ctx.beginPath();for(var i=0;i<6;i++){var a=(Math.PI/3)*i-(Math.PI/6);",
            "  var x=cx+r*Math.cos(a),y=cy+r*Math.sin(a);if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);}ctx.closePath();}",
            " function animate(){ctx.clearRect(0,0,width,height);ctx.lineWidth=0.5;",
            "  var dark=document.documentElement.getAttribute('data-theme')==='dark';",
            "  var base=dark?0.02:0.05,boost=dark?0.28:0.22,rgb=dark?'255,255,255':'0,0,0';",
            "  var cols=Math.ceil(width/hexWidth)+2,rows=Math.ceil(height/vertDist)+2;",
            "  for(var row=-1;row<rows;row++){for(var col=-1;col<cols;col++){",
            "   var cx=col*hexWidth,cy=row*vertDist;if(row%2!==0)cx+=hexWidth/2;",
            "   var alpha=base;",
            "   if(mouse.active){var dx=mouse.x-cx,dy=mouse.y-cy,d=Math.sqrt(dx*dx+dy*dy);",
            "    if(d<120)alpha+=(1-(d/120))*boost;}",
            "   ctx.strokeStyle='rgba('+rgb+','+alpha+')';drawHexagon(cx,cy,hexRadius);ctx.stroke();}}",
            "  requestAnimationFrame(animate);}",
            " animate();}",
            "initHoneycomb();",
            "setInterval(poll,250);poll();",
            "</script></body></html>"
    });

    private static String join(String[] lines) {
        StringBuilder sb = new StringBuilder(16 * 1024);
        for (String line : lines) {
            sb.append(line).append('\n');
        }
        return sb.toString();
    }
}
