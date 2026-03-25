package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class Constants {

    public static final class VisionConstants {
        // Camera offset from robot center of rotation.
        // WPILib convention: x = forward, y = left, z = up.
        // All values in meters (converted from inches: in × 0.0254).
        public static final double kCamX = -7.75 * 0.0254; // 7.75 in behind center
        public static final double kCamY =  8.5  * 0.0254; //  8.5 in left of center
        public static final double kCamZ = 16.5  * 0.0254; // 16.5 in above center

        public static final String kCamName = "front-cam";

        // Camera is mounted level — no roll, pitch, or yaw correction.
        public static final Transform3d kRobotToCam = new Transform3d(
            new Translation3d(kCamX, kCamY, kCamZ),
            new Rotation3d(0.0, 0.0, 0.0)
        );
    }
}
