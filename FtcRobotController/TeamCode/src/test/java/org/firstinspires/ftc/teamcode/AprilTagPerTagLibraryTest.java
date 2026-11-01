package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.apriltag.AprilTagClusterMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * The webcam detector must report tags one by one, not one per hive cell.
 *
 * <p>SDK 12 groups a cell's four tags into a cluster and emits a single {@code
 * AprilTagClusterDetection} for them - one dashboard row, no corners, no per-tag pose. The OpMode
 * therefore runs the detector with {@code AprilTagDashboardOpMode.perTagLibrary()}, which
 * re-registers every cluster member as a plain tag. These tests hold that library to the two
 * things the dashboard needs from it: no clusters left to fold tags into, and every detected tag
 * still carrying the metadata - name and real size - that a pose solve needs.
 */
class AprilTagPerTagLibraryTest {

    /** The SDK's own tag libraries top out here; scanning past it costs nothing. */
    private static final int MAX_TAG_ID = 64;

    @Test
    void everyClusterTagBecomesATagOfItsOwn() {
        AprilTagLibrary game = AprilTagGameDatabase.getCurrentGameTagLibrary();
        AprilTagLibrary perTag = AprilTagDashboardOpMode.perTagLibrary();

        assertEquals(0, perTag.getAllClusters().length,
                "a cluster left in the library is exactly what swallows four tags into one row");

        int checked = 0;
        for (int id = 0; id < MAX_TAG_ID; id++) {
            AprilTagClusterMetadata cluster = game.lookupCluster(id);
            if (cluster == null) {
                continue;
            }
            checked++;
            assertNull(perTag.lookupCluster(id),
                    "tag " + id + " is still folded into " + cluster.name);
            AprilTagMetadata tag = perTag.lookupTag(id);
            assertNotNull(tag,
                    "tag " + id + " has no metadata, so the detector would give it no pose");
            assertEquals(id, tag.id);
            assertTrue(tag.tagsize > 0, "tag " + id + " has no size, so its range is meaningless");
            char alliance = Character.toUpperCase(tag.name.charAt(0));
            assertTrue(alliance == 'R' || alliance == 'B',
                    "the alliance rule reads the first letter of the name, got '" + tag.name + "'");
        }
        assertTrue(checked >= 4,
                "expected the game's clustered tags to be found, saw " + checked);
    }

    @Test
    void theGameTagsKeepTheirSdkNamesAndSizes() {
        AprilTagLibrary game = AprilTagGameDatabase.getCurrentGameTagLibrary();
        AprilTagLibrary perTag = AprilTagDashboardOpMode.perTagLibrary();

        AprilTagMetadata[] tags = game.getAllTags();
        assertTrue(tags.length > 0, "the game library must have tags to keep");
        for (AprilTagMetadata tag : tags) {
            AprilTagMetadata kept = perTag.lookupTag(tag.id);
            assertNotNull(kept, "tag " + tag.id + " disappeared from the per-tag library");
            assertEquals(tag.name, kept.name, "tag " + tag.id + " was renamed");
            assertEquals(tag.tagsize, kept.tagsize, 1e-9, "tag " + tag.id + " changed size");
        }
    }
}
