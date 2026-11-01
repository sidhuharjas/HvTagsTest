package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfInt;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.Locale;

/**
 * Feeds the dashboard's Live view: turns the frames the VisionPortal already decodes into JPEG
 * for {@code GET /stream.mjpg}, with a box around every tag the OpMode has detected.
 *
 * <p>It runs on the camera thread beside the AprilTag processor, so it does nothing at all until
 * a browser is watching ({@link AprilTagDashboardServer#wantsFrames()}) and caps itself at about
 * 30 encoded frames a second - the camera's own rate, so the picture is not held back by the
 * encoding. The encoding is deliberately downscaled in quality, not resolution:
 * a 640x480 frame at 70 is a few tens of kB, which is what a pit laptop on the robot's wifi can
 * hold while the JSON polling continues.
 *
 * <p>Added before the AprilTag processor, so the frame it boxes is still the untouched one. That
 * also means the boxes come from the detections of the frame before this one - a tag box trails
 * the tag by the one camera frame between them, which is what every on-screen overlay does.
 */
public class MjpegFrameGrabber implements VisionProcessor {

    /** ~30 fps: match the camera rather than add a second of buffering to what you see. */
    private static final long MIN_ENCODE_INTERVAL_NS = 33_000_000L;
    private static final int JPEG_QUALITY = 70;
    /** BGR: the frame being encoded is BGR, not RGB. */
    private static final Scalar TAG_COLOR = new Scalar(0, 255, 0);
    private static final Scalar LABEL_SHADOW = new Scalar(0, 0, 0);

    private volatile AprilTagDashboardServer server;
    /** The OpMode's latest detections, so the picture shows where the tags are. */
    private volatile List<AprilTagDetection> detections = java.util.Collections.emptyList();
    private final Mat bgr = new Mat();
    private final MatOfByte jpeg = new MatOfByte();
    private final MatOfInt quality = new MatOfInt(Imgcodecs.IMWRITE_JPEG_QUALITY, JPEG_QUALITY);
    private long lastEncodeNanos;
    private volatile String lastError = "";

    /** The dashboard server that serves the frames; null until the OpMode has created it. */
    public void setServer(AprilTagDashboardServer server) {
        this.server = server;
    }

    /**
     * The detections to box on the next encoded frame, replaced once per OpMode loop.
     *
     * <p>A whole list, never edited in place: the camera thread reads it while the loop thread
     * is building the next one, and swapping the reference is the only synchronisation needed.
     */
    public void setDetections(List<AprilTagDetection> detections) {
        this.detections = detections == null
                ? java.util.Collections.<AprilTagDetection>emptyList() : detections;
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
            annotate();
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

    /**
     * Outlines every detected tag on {@link #bgr} and labels it with its id and range.
     *
     * <p>Only single detections have corners - a cluster detection carries a pose for the whole
     * cluster and none of the tags' pixel positions - so those are the ones that can be boxed.
     */
    private void annotate() {
        List<AprilTagDetection> current = detections;
        if (current.isEmpty()) {
            return;
        }
        double scale = bgr.cols() >= 1000 ? 0.8 : 0.5;
        for (AprilTagDetection d : current) {
            if (!(d instanceof AprilTagSingleDetection)) {
                continue;
            }
            AprilTagSingleDetection tag = (AprilTagSingleDetection) d;
            Point[] corners = tag.corners;
            if (corners == null || corners.length < 4) {
                continue;
            }
            double left = corners[0].x;
            double top = corners[0].y;
            for (int i = 0; i < 4; i++) {
                Imgproc.line(bgr, corners[i], corners[(i + 1) % 4], TAG_COLOR, 2, Imgproc.LINE_AA, 0);
                left = Math.min(left, corners[i].x);
                top = Math.min(top, corners[i].y);
            }
            String label = tag.ftcPose == null
                    ? "ID " + tag.id
                    : String.format(Locale.US, "ID %d  %.0f in", tag.id, tag.ftcPose.range);
            // Kept inside the frame: a tag at the very top would otherwise label off-image.
            Point at = new Point(left, Math.max(top - 6, 14));
            Imgproc.putText(bgr, label, new Point(at.x + 1, at.y + 1),
                    Imgproc.FONT_HERSHEY_SIMPLEX, scale, LABEL_SHADOW, 3, Imgproc.LINE_AA, false);
            Imgproc.putText(bgr, label, at,
                    Imgproc.FONT_HERSHEY_SIMPLEX, scale, TAG_COLOR, 1, Imgproc.LINE_AA, false);
        }
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        // The frames are shown in the browser, not on a phone screen.
    }
}
