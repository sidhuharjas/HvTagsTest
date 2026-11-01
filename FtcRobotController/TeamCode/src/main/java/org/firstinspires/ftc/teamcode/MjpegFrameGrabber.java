package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfInt;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

/**
 * Feeds the dashboard's Live view: turns the frames the VisionPortal already decodes into JPEG
 * for {@code GET /stream.mjpg}.
 *
 * <p>It runs on the camera thread beside the AprilTag processor, so it does nothing at all until
 * a browser is watching ({@link AprilTagDashboardServer#wantsFrames()}) and caps itself at about
 * 15 encoded frames a second. The encoding is deliberately downscaled in quality, not resolution:
 * a 640x480 frame at 70 is a few tens of kB, which is what a pit laptop on the robot's wifi can
 * hold while the JSON polling continues.
 *
 * <p>Added before the AprilTag processor, so the frame it encodes is still the untouched one.
 */
public class MjpegFrameGrabber implements VisionProcessor {

    /** ~15 fps. The dashboard is a tuning view; the Driver Station live view is the smooth one. */
    private static final long MIN_ENCODE_INTERVAL_NS = 66_000_000L;
    private static final int JPEG_QUALITY = 70;

    private volatile AprilTagDashboardServer server;
    private final Mat bgr = new Mat();
    private final MatOfByte jpeg = new MatOfByte();
    private final MatOfInt quality = new MatOfInt(Imgcodecs.IMWRITE_JPEG_QUALITY, JPEG_QUALITY);
    private long lastEncodeNanos;
    private volatile String lastError = "";

    /** The dashboard server that serves the frames; null until the OpMode has created it. */
    public void setServer(AprilTagDashboardServer server) {
        this.server = server;
    }

    /** {@return why the last frame could not be encoded, or "" when encoding works}. */
    public String error() {
        return lastError;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // No per-frame state depends on the size; the portal may change it on a rebuild.
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        AprilTagDashboardServer s = server;
        if (s == null || !s.wantsFrames() || frame == null || frame.empty()) {
            return null; // nobody is watching: leave the loop budget to AprilTag
        }
        long now = System.nanoTime();
        if (now - lastEncodeNanos < MIN_ENCODE_INTERVAL_NS) {
            return null;
        }
        lastEncodeNanos = now;
        try {
            // VisionPortal hands processors an RGBA frame; JPEG wants 3 channels.
            Imgproc.cvtColor(frame, bgr, Imgproc.COLOR_RGBA2BGR);
            Imgcodecs.imencode(".jpg", bgr, jpeg, quality);
            if (!jpeg.empty()) {
                s.publishFrame(jpeg.toArray());
                lastError = "";
            }
        } catch (Throwable e) {
            // A broken encoder must never end the match; the page shows "no frames" instead.
            lastError = "video: " + e;
        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        // The frames are shown in the browser, not on a phone screen.
    }
}
