package org.firstinspires.ftc.teamcode;

import org.json.JSONObject;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * The tunable config the dashboard posts to: defaults, the dirty flag the OpMode loop polls,
 * clamping of hostile slider values, the JSON round trip, and the hive-cell tip check.
 *
 * <p>Runs against the real {@code org.json} SimLoop puts on the test classpath, which is what the
 * Android runtime provides on the robot - Android's own copy is a stub in a JVM test.
 */
class AprilTagConfigTest {

    @Test
    void defaultsMatchWhatTheDashboardShows() {
        AprilTagConfig config = new AprilTagConfig();

        assertEquals("Webcam 1", config.cameraName);
        assertEquals(0, config.resolution);
        assertEquals(1f, config.decimation, 1e-6,
                "decimation defaults to full resolution: halving it dropped tags off the list");
        assertEquals(5.0, config.tagSizeIn, 1e-9);
        assertEquals(65.0, config.fovDeg, 1e-9);
        assertEquals("RED", config.alliance);
        assertEquals(90.0, config.maxRollDeg, 1e-9);
        assertEquals(AprilTagConfig.POSE_SOLVER_DEFAULT, config.poseSolver);
        assertEquals(AprilTagConfig.EXPOSURE_AUTO, config.exposureUs);
        assertEquals(AprilTagConfig.GAIN_UNCHANGED, config.gain);
    }

    @Test
    void dirtyFlagFiresOncePerChange() {
        AprilTagConfig config = new AprilTagConfig();

        assertTrue(config.consumeDirty(), "a fresh config has never been applied to the camera");
        assertFalse(config.consumeDirty(), "it must not re-apply every loop");

        config.applyJson("{\"decimation\":2}");
        assertTrue(config.consumeDirty());
        assertFalse(config.consumeDirty());
    }

    @Test
    void aPartialPostLeavesEverythingElseAlone() {
        AprilTagConfig config = new AprilTagConfig();

        config.applyJson("{\"alliance\":\"BLUE\",\"tagSizeIn\":8.5}");

        assertEquals("BLUE", config.alliance);
        assertEquals(8.5, config.tagSizeIn, 1e-9);
        assertEquals("Webcam 1", config.cameraName, "untouched keys keep their value");
        assertEquals(65.0, config.fovDeg, 1e-9);
        assertEquals(1f, config.decimation, 1e-6);
    }

    @Test
    void outOfRangeSliderValuesAreClamped() {
        AprilTagConfig config = new AprilTagConfig();

        config.applyJson("{\"tagSizeIn\":9999,\"fovDeg\":1,\"resolution\":42,\"minClusterPercent\":500}");
        assertEquals(48.0, config.tagSizeIn, 1e-9, "tag size cannot leave the supported band");
        assertEquals(20.0, config.fovDeg, 1e-9);
        assertEquals(AprilTagConfig.RES_W.length - 1, config.resolution);
        assertEquals(100, config.minClusterPercent);

        config.applyJson("{\"tagSizeIn\":-5,\"fovDeg\":1000}");
        assertEquals(0.1, config.tagSizeIn, 1e-9);
        assertEquals(170.0, config.fovDeg, 1e-9);
    }

    @Test
    void malformedJsonNeverThrowsAndNeverWipesTheConfig() {
        AprilTagConfig config = new AprilTagConfig();
        config.applyJson("{\"alliance\":\"BLUE\"}");

        config.applyJson("{not json at all");
        config.applyJson(null);
        config.applyJson("");

        assertEquals("BLUE", config.alliance,
                "a broken POST from a half-typed slider must not reset the robot");
    }

    @Test
    void jsonRoundTripPreservesEveryField() throws Exception {
        AprilTagConfig original = new AprilTagConfig();
        original.applyJson("{\"cameraName\":\"BACK\",\"resolution\":1,\"decimation\":3.5,"
                + "\"tagSizeIn\":7.25,\"fovDeg\":90,\"exposureUs\":5000,\"gain\":40,"
                + "\"alliance\":\"BLUE\",\"maxRollDeg\":45,\"minClusterPercent\":60,"
                + "\"minDecisionMargin\":25,\"poseSolver\":\"OPENCV_IPPE_SQUARE\","
                + "\"showUnscorable\":false,\"liveView\":false,"
                + "\"hiveBaselinePitchDeg\":12.5,\"hiveTippedThresholdDeg\":30}");

        JSONObject sent = new JSONObject(original.toJson());
        AprilTagConfig received = new AprilTagConfig();
        received.applyJson(sent.toString());

        assertEquals(original.cameraName, received.cameraName);
        assertEquals(original.resolution, received.resolution);
        assertEquals(original.decimation, received.decimation, 1e-6);
        assertEquals(original.tagSizeIn, received.tagSizeIn, 1e-9);
        assertEquals(original.fovDeg, received.fovDeg, 1e-9);
        assertEquals(original.exposureUs, received.exposureUs);
        assertEquals(original.gain, received.gain);
        assertEquals(original.alliance, received.alliance);
        assertEquals(original.maxRollDeg, received.maxRollDeg, 1e-9);
        assertEquals(original.minClusterPercent, received.minClusterPercent);
        assertEquals(original.minDecisionMargin, received.minDecisionMargin, 1e-6);
        assertEquals(original.poseSolver, received.poseSolver);
        assertEquals(original.showUnscorable, received.showUnscorable);
        assertEquals(original.liveView, received.liveView);
        assertEquals(original.hiveBaselinePitchDeg, received.hiveBaselinePitchDeg, 1e-9);
        assertEquals(original.hiveTippedThresholdDeg, received.hiveTippedThresholdDeg, 1e-9);
    }

    @Test
    void resetReturnsEveryFieldToItsDefault() {
        AprilTagConfig config = new AprilTagConfig();
        config.applyJson("{\"decimation\":4,\"tagSizeIn\":20,\"alliance\":\"BLUE\",\"fovDeg\":120}");

        config.resetToDefaults();

        assertEquals(1f, config.decimation, 1e-6);
        assertEquals(5.0, config.tagSizeIn, 1e-9);
        assertEquals(65.0, config.fovDeg, 1e-9);
        assertEquals("RED", config.alliance);
        assertTrue(config.consumeDirty(), "resetting must re-apply to the camera");
    }

    @Test
    void hiveTipCheckUsesTheBaselineAndTheThreshold() {
        assertFalse(AprilTagConfig.isTipped(0.0, 0.0, 25.0),
                "an upright cell at the baseline is upright");
        assertFalse(AprilTagConfig.isTipped(Double.NaN, 0.0, 25.0),
                "no detection reads as not tipped, never as tipped");

        assertTrue(AprilTagConfig.isTipped(30.0, 0.0, 25.0),
                "30 deg from a 0 deg baseline is past a 25 deg threshold");
        assertFalse(AprilTagConfig.isTipped(24.0, 0.0, 25.0));
        assertFalse(AprilTagConfig.isTipped(-24.0, 0.0, 25.0),
                "the check is on |pitch - baseline|, so either direction counts");

        assertTrue(AprilTagConfig.isTipped(56.0, 30.0, 25.0),
                "the baseline moves with the cell, so 56 is 26 from a 30 degree baseline");
        assertFalse(AprilTagConfig.isTipped(54.0, 30.0, 25.0));
    }

    @Test
    void theSourceIsChosenByTheDashboardAndNormalizedOnTheWayIn() {
        AprilTagConfig config = new AprilTagConfig();
        assertFalse(config.isLimelight(), "the default source is the USB webcam");

        assertEquals(AprilTagConfig.MODE_WEBCAM, AprilTagConfig.normalizeMode(null));
        assertEquals(AprilTagConfig.MODE_WEBCAM, AprilTagConfig.normalizeMode("banana"));
        assertEquals(AprilTagConfig.MODE_LIMELIGHT, AprilTagConfig.normalizeMode("limelight"));
        assertEquals(AprilTagConfig.MODE_LIMELIGHT, AprilTagConfig.normalizeMode(" Limelight "));

        config.applyJson("{\"cameraMode\":\"LIMELIGHT\"}");
        assertTrue(config.isLimelight());
        assertTrue(config.consumeDirty(), "changing the source has to reach the loop thread");

        config.applyJson("{\"cameraMode\":\"usb\"}");
        assertEquals(AprilTagConfig.MODE_WEBCAM, config.cameraMode);
        assertFalse(config.isLimelight());
    }

    @Test
    void limelightSettingsClampAndRoundTrip() throws Exception {
        AprilTagConfig config = new AprilTagConfig();

        config.applyJson("{\"limelightName\":\"  LL-3A \",\"limelightPollHz\":999}");
        assertEquals("LL-3A", config.limelightName, "the device name is trimmed like the camera one");
        assertEquals(120, config.limelightPollHz);

        config.applyJson("{\"limelightPollHz\":0}");
        assertEquals(1, config.limelightPollHz, "a poll rate below 1 Hz is not a poll rate");

        config.applyJson("{\"cameraMode\":\"LIMELIGHT\",\"limelightName\":\"ll2\",\"limelightPollHz\":30}");
        JSONObject sent = new JSONObject(config.toJson());
        assertEquals("LIMELIGHT", sent.getString("cameraMode"));
        assertEquals("ll2", sent.getString("limelightName"));
        assertEquals(30, sent.getInt("limelightPollHz"));

        assertEquals("limelight.local", AprilTagConfig.sanitizeHost("  https://limelight.local:5801/ "),
                "a pasted browser URL is cut down to the host itself");
        assertEquals("10.0.0.5", AprilTagConfig.sanitizeHost("10.0.0.5:5807"),
                "the port is the caller's business, not the host's");
        assertEquals("", AprilTagConfig.sanitizeHost(null), "nothing typed, nothing to reach");

        config.applyJson("{\"limelightHost\":\"https://hive-cam.local:5807/\"}");
        assertEquals("hive-cam.local", config.limelightHost);
        assertEquals("hive-cam.local", new JSONObject(config.toJson()).getString("limelightHost"),
                "a dashboard that reloads still shows the host it was given");

        config.resetToDefaults();
        assertEquals(AprilTagConfig.MODE_WEBCAM, config.cameraMode);
        assertEquals("limelight", config.limelightName);
        assertEquals(100, config.limelightPollHz);
        assertEquals("", config.limelightHost, "reset goes back to asking the device");
    }

    @Test
    void pushTagSizeIsAOneShotRequestAndNeverRoundTrips() throws Exception {
        AprilTagConfig config = new AprilTagConfig();
        assertFalse(config.consumePushTagSize(), "nothing has asked for a push yet");

        config.applyJson("{\"pushTagSize\":true}");
        assertTrue(config.consumePushTagSize(), "the dashboard's button reaches the loop thread");
        assertFalse(config.consumePushTagSize(), "exactly once: one button press, one round-trip");

        config.applyJson("{\"pushTagSize\":true}");
        assertFalse(new JSONObject(config.toJson()).has("pushTagSize"),
                "a request is an event, not a setting: the state must not echo it back");
        assertTrue(config.consumePushTagSize());

        config.applyJson("{\"pushTagSize\":false}");
        assertFalse(config.consumePushTagSize(), "a false must never fire the push");

        config.applyJson("{\"pushTagSize\":true}");
        config.resetToDefaults();
        assertFalse(config.consumePushTagSize(), "resetting drops a pending request");
    }

    @Test
    void tagNamesMapIdsToNamesForTheNamelessLimelightResults() {
        AprilTagConfig config = new AprilTagConfig();
        assertNull(config.tagName(1), "no names by default, so the alliance rule stays open");

        config.applyJson("{\"tagNames\":\"{\\\"1\\\":\\\"R_HIVE\\\"}\"}");
        assertEquals("R_HIVE", config.tagName(1));
        assertNull(config.tagName(2));
        assertEquals(1, config.tagNameMap().size());

        // The dashboard may POST the map as an object rather than a pre-encoded string.
        config.applyJson("{\"tagNames\":{\"2\":\"B_CELL\"}}");
        assertEquals("B_CELL", config.tagName(2));
        assertNull(config.tagName(1), "a new map replaces the old one");

        config.applyJson("{\"tagNames\":\"not an object\"}");
        assertEquals("B_CELL", config.tagName(2), "a broken map is never stored");

        config.applyJson("{\"tagNames\":\"{\\\"2\\\":\\\"B_TAG\\\"}\"}");
        assertEquals("B_TAG", config.tagName(2), "an edit must not serve the cached name");

        config.resetToDefaults();
        assertNull(config.tagName(2));
    }
}
