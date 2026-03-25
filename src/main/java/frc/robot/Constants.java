package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class Constants {

    public static final class VisionConstants {
        // Camera offsets from robot center of rotation.
        // WPILib convention: x = forward, y = left, z = up.
        // All values in meters (converted from inches: in × 0.0254).

        // --- Front camera ---
        public static final String kFrontCamName = "front-cam";
        public static final double kFrontCamX = -7.75 * 0.0254; // 7.75 in behind center
        public static final double kFrontCamY =  8.5  * 0.0254; //  8.5 in left of center
        public static final double kFrontCamZ = 16.5  * 0.0254; // 16.5 in above center
        // Mounted level — no roll, pitch, or yaw correction.
        public static final Transform3d kFrontRobotToCam = new Transform3d(
            new Translation3d(kFrontCamX, kFrontCamY, kFrontCamZ),
            new Rotation3d(0.0, 0.0, 0.0)
        );

        // --- Side camera (facing left) ---
        public static final String kSideCamName = "side-cam";
        public static final double kSideCamX =  2.5 * 0.0254; //  2.5 in forward of center
        public static final double kSideCamY = 11.0 * 0.0254; // 11.0 in left of center
        public static final double kSideCamZ =  7.0 * 0.0254; //  7.0 in above center
        // Rotated 90° left (yaw = π/2), mounted level otherwise.
        public static final Transform3d kSideRobotToCam = new Transform3d(
            new Translation3d(kSideCamX, kSideCamY, kSideCamZ),
            new Rotation3d(0.0, 0.0, Math.PI / 2.0)
        );
    }
}
