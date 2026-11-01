package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;
import java.util.Locale;

/**
 * Minimal webcam AprilTag check - no dashboard, just telemetry. Use this first to confirm
 * the camera streams and tags are detectable, then switch to AprilTagDashboardOpMode.
 *
 * Shows both SDK 12 detection kinds: AprilTagSingleDetection (standalone tags) and
 * AprilTagClusterDetection (one detection per cluster, no matter how many member tags are
 * visible). Cluster detections always have metadata; single detections may not.
 *
 * Requires FTC SDK 12.0+.
 */
@TeleOp(name = "AprilTag Telemetry (Webcam)", group = "Vision")
public class AprilTagTelemetryOpMode extends OpMode {

    private static final String CAM_NAME = "Webcam 1";

    private AprilTagProcessor aprilTag;
    private VisionPortal portal;

    @Override
    public void init() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(false)
                .setDrawTagID(false)
                .build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAM_NAME))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(false)
                .addProcessor(aprilTag)
                .build();
    }

    @Override
    public void loop() {
        try {
            loopOnce();
        } catch (Throwable e) {
            // Same rule as the dashboard OpMode: a detection that cannot be read must not end
            // the run, and the reason has to reach the Driver Station.
            telemetry.addLine("loop failed - " + e);
        }
    }

    private void loopOnce() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        telemetry.addData("cam", portal.getCameraState().name());
        telemetry.addData("fps", "%.1f", portal.getFps());
        telemetry.addData("detections", detections.size());

        for (AprilTagDetection detection : detections) {
            telemetry.addLine("");

            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                telemetry.addLine(String.format(Locale.US, "==== CLUSTER %s (%d%% visible)",
                        cluster.metadata == null ? "?" : cluster.metadata.name,
                        cluster.percentClusterFound));
            } else if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
                if (single.metadata != null) {
                    telemetry.addLine(String.format(Locale.US, "==== (ID %d) %s",
                            single.id, single.metadata.name));
                } else {
                    telemetry.addLine(String.format(Locale.US, "==== (ID %d) no metadata", single.id));
                }
            }

            // Pose is on the shared superclass, so it reads the same either way - and it can be
            // null when the SDK could not solve it, which must not take the OpMode down.
            if (detection.ftcPose == null) {
                telemetry.addLine("no pose for this tag (poseless detection)");
                continue;
            }
            telemetry.addLine(String.format(Locale.US, "XYZ %6.1f %6.1f %6.1f  (inch)",
                    detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
            telemetry.addLine(String.format(Locale.US, "PRY %6.1f %6.1f %6.1f  (deg)",
                    detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
            telemetry.addLine(String.format(Locale.US, "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                    detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
                telemetry.addLine(String.format(Locale.US, "hamming %d  margin %.1f",
                        single.hamming, single.decisionMargin));
            }
        }
    }

    @Override
    public void stop() {
        if (portal != null) {
            portal.close();
            portal = null;
        }
    }
}
