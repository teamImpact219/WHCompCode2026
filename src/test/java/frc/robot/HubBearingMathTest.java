package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

/**
 * Tests the hub bearing calculation used in FaceHubCommand.computePoseFallbackHeading.
 *
 * The formula is:  bearing = atan2(hubY - robotY, hubX - robotX)
 *
 * This is a pure-math test — no HAL, no hardware, no subsystem instantiation needed.
 * If the formula in FaceHubCommand ever changes, update these tests to match.
 */
class HubBearingMathTest {

    /** Mirrors FaceHubCommand.computePoseFallbackHeading math for a single hub point. */
    private static Rotation2d bearing(double robotX, double robotY, double hubX, double hubY) {
        return new Rotation2d(Math.atan2(hubY - robotY, hubX - robotX));
    }

    @Test
    void hubDirectlyAhead_fieldForwardIsEast() {
        // Robot at origin facing east (+X), hub also on +X axis → bearing = 0°
        Rotation2d result = bearing(0, 0, 5, 0);
        assertEquals(0.0, result.getDegrees(), 1e-9, "Hub due east → 0°");
    }

    @Test
    void hubDirectlyNorth() {
        // Hub is at +Y from robot → atan2(+5, 0) = 90°
        Rotation2d result = bearing(0, 0, 0, 5);
        assertEquals(90.0, result.getDegrees(), 1e-9, "Hub due north → 90°");
    }

    @Test
    void hubNorthEast45() {
        Rotation2d result = bearing(0, 0, 5, 5);
        assertEquals(45.0, result.getDegrees(), 1e-9, "Hub NE → 45°");
    }

    @Test
    void hubDirectlyBehind() {
        // Hub is at -X from robot → 180°
        Rotation2d result = bearing(0, 0, -5, 0);
        assertEquals(180.0, Math.abs(result.getDegrees()), 1e-9, "Hub due west → ±180°");
    }

    @Test
    void robotOffsetFromOrigin() {
        // Robot at (3, 3), hub at (8, 3) — hub is due east
        Rotation2d result = bearing(3, 3, 8, 3);
        assertEquals(0.0, result.getDegrees(), 1e-9, "Offset robot, hub due east → 0°");
    }

    @Test
    void averagedHubPosition() {
        // Simulates averaging two tag positions (as FaceHubCommand does for the fallback)
        // Tag A at (16.0, 4.0), Tag B at (16.0, 6.0) → hub center = (16.0, 5.0)
        double tagAx = 16.0, tagAy = 4.0;
        double tagBx = 16.0, tagBy = 6.0;
        double hubX = (tagAx + tagBx) / 2.0;  // 16.0
        double hubY = (tagAy + tagBy) / 2.0;  // 5.0

        Rotation2d result = bearing(0, 5, hubX, hubY);
        // Robot at (0, 5), hub at (16, 5) → due east = 0°
        assertEquals(0.0, result.getDegrees(), 1e-9, "Hub center due east → 0°");
    }
}
