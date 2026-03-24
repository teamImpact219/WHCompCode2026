package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import frc.robot.subsystems.VisionSubsystem;

/**
 * Verifies that the 2026 REBUILT field layout loads correctly and contains
 * the hub AprilTag IDs used by VisionSubsystem and FaceHubCommand.
 *
 * Pure data test — no HAL or hardware initialization needed.
 */
class FieldLayoutTest {

    private static AprilTagFieldLayout layout;

    @BeforeAll
    static void loadLayout() {
        layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    }

    @Test
    void layoutLoads() {
        assertNotNull(layout, "Field layout must load from WPILib resources");
        assertFalse(layout.getTags().isEmpty(), "Field layout must contain at least one tag");
    }

    @Test
    void redHubTagsExistInLayout() {
        for (int id : VisionSubsystem.RED_HUB_TAG_IDS) {
            assertTrue(layout.getTagPose(id).isPresent(),
                "Red hub tag " + id + " not found in 2026 REBUILT layout — check game manual");
        }
    }

    @Test
    void blueHubTagsExistInLayout() {
        for (int id : VisionSubsystem.BLUE_HUB_TAG_IDS) {
            assertTrue(layout.getTagPose(id).isPresent(),
                "Blue hub tag " + id + " not found in 2026 REBUILT layout — check game manual");
        }
    }

    @Test
    void redHubTagsHaveReasonablePositions() {
        // Tags should be on the field (roughly 0-17m x, 0-8m y for 2026)
        for (int id : VisionSubsystem.RED_HUB_TAG_IDS) {
            var pose = layout.getTagPose(id).orElseThrow();
            assertTrue(pose.getX() >= 0 && pose.getX() <= 18,
                "Tag " + id + " X position out of expected field bounds: " + pose.getX());
            assertTrue(pose.getY() >= 0 && pose.getY() <= 9,
                "Tag " + id + " Y position out of expected field bounds: " + pose.getY());
        }
    }

    @Test
    void blueHubTagsHaveReasonablePositions() {
        for (int id : VisionSubsystem.BLUE_HUB_TAG_IDS) {
            var pose = layout.getTagPose(id).orElseThrow();
            assertTrue(pose.getX() >= 0 && pose.getX() <= 18,
                "Tag " + id + " X position out of expected field bounds: " + pose.getX());
            assertTrue(pose.getY() >= 0 && pose.getY() <= 9,
                "Tag " + id + " Y position out of expected field bounds: " + pose.getY());
        }
    }
}
