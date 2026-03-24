package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.VisionSubsystem;

/**
 * Verifies that VisionSubsystem public constants have the expected values.
 * If these change (e.g. a camera is renamed), tests will fail immediately
 * rather than causing a silent runtime mismatch.
 */
class VisionConstantsTest {

    @Test
    void redHubTagIds() {
        assertArrayEquals(new int[]{9, 10}, VisionSubsystem.RED_HUB_TAG_IDS,
            "Red hub tag IDs should be 9 and 10 for 2026 REBUILT");
    }

    @Test
    void blueHubTagIds() {
        assertArrayEquals(new int[]{25, 26}, VisionSubsystem.BLUE_HUB_TAG_IDS,
            "Blue hub tag IDs should be 25 and 26 for 2026 REBUILT");
    }

    @Test
    void frontCameraName() {
        assertEquals("front-cam", VisionSubsystem.FRONT_CAMERA_NAME,
            "Camera name must match exactly what is set in the PhotonVision web UI");
    }

    @Test
    void hubTagArraysNotEmpty() {
        assertTrue(VisionSubsystem.RED_HUB_TAG_IDS.length > 0);
        assertTrue(VisionSubsystem.BLUE_HUB_TAG_IDS.length > 0);
    }
}
