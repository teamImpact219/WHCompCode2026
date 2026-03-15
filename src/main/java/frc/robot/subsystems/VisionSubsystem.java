package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {

    // -----------------------------------------------------------------------
    // Camera configuration — names MUST match what is set in PhotonVision UI
    // -----------------------------------------------------------------------

    // TODO (BEFORE DEPLOYMENT): Verify these names match exactly what you named
    // each camera in the PhotonVision web UI on the Orange Pi.
    // Open a browser to http://photonvision.local:5800 and check the camera names.
    public static final String FRONT_CAMERA_NAME = "front_camera";
    public static final String SIDE_CAMERA_NAME  = "side_camera";

    // Hub AprilTag IDs for each alliance (2026 REBUILT game)
    public static final int[] RED_HUB_TAG_IDS  = {9, 10};
    public static final int[] BLUE_HUB_TAG_IDS = {25, 26};

    // -----------------------------------------------------------------------
    // Camera mount transforms: robot center → camera lens.
    // Coordinate system: +X forward, +Y left, +Z up (WPILib convention).
    // Translation3d(forward_m, left_m, up_m)
    // Rotation3d(roll_rad, pitch_rad, yaw_rad) — all zero = camera faces straight forward
    // -----------------------------------------------------------------------

    // TODO (BEFORE DEPLOYMENT): Measure ROBOT_TO_FRONT_CAMERA from your robot's
    // CAD or physical build and replace the placeholder values below.
    //   - Translation: how far forward, left, and up the camera lens is from the
    //     center of the robot (in meters).
    //   - Rotation: camera roll/pitch/yaw relative to the robot's forward axis.
    //     Zero rotation means the camera faces straight forward with no tilt.
    // Current placeholder: 30 cm forward, centered, 50 cm up, facing straight forward.
    public static final Transform3d ROBOT_TO_FRONT_CAMERA = new Transform3d(
        new Translation3d(0.30, 0.0, 0.50),
        new Rotation3d(0, 0, 0)
    );

    // TODO (BEFORE DEPLOYMENT): Measure ROBOT_TO_SIDE_CAMERA from your robot's
    // CAD or physical build and replace the placeholder values below.
    //   - The side camera faces 90° to the left of robot forward (yaw = +90°).
    //   - Adjust the Translation3d to match where the camera is actually mounted.
    // Current placeholder: centered fore/aft, 25 cm left of center, 50 cm up, facing left.
    public static final Transform3d ROBOT_TO_SIDE_CAMERA = new Transform3d(
        new Translation3d(0.0, 0.25, 0.50),
        new Rotation3d(0, 0, Math.toRadians(90))
    );

    // -----------------------------------------------------------------------
    // Vision pose measurement standard deviations [x, y, theta].
    // Lower values = trust vision more. Increase if measurements are noisy.
    // -----------------------------------------------------------------------
    private static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4.0, 4.0, 8.0);
    private static final Matrix<N3, N1> MULTI_TAG_STD_DEVS  = VecBuilder.fill(0.5, 0.5, 1.0);

    // -----------------------------------------------------------------------
    // Internal state
    // -----------------------------------------------------------------------
    private final CommandSwerveDrivetrain drivetrain;
    private final PhotonCamera frontCamera;
    private final PhotonCamera sideCamera;
    private final AprilTagFieldLayout fieldLayout;
    private final PhotonPoseEstimator frontEstimator;
    private final PhotonPoseEstimator sideEstimator;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        frontCamera = new PhotonCamera(FRONT_CAMERA_NAME);
        sideCamera  = new PhotonCamera(SIDE_CAMERA_NAME);

        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        frontEstimator = new PhotonPoseEstimator(fieldLayout, ROBOT_TO_FRONT_CAMERA);
        sideEstimator  = new PhotonPoseEstimator(fieldLayout, ROBOT_TO_SIDE_CAMERA);
    }

    @Override
    public void periodic() {
        updatePoseEstimates(frontCamera, frontEstimator);
        updatePoseEstimates(sideCamera, sideEstimator);
    }

    /**
     * Returns the best visible hub AprilTag target for the current alliance,
     * preferring the front camera over the side camera.
     * "Best" means the target with the lowest pose ambiguity among hub tags.
     */
    public Optional<PhotonTrackedTarget> getBestHubTarget() {
        int[] hubIds = getAllianceHubTagIds();

        Optional<PhotonTrackedTarget> frontTarget = getBestTargetFromCamera(frontCamera, hubIds);
        if (frontTarget.isPresent()) return frontTarget;

        return getBestTargetFromCamera(sideCamera, hubIds);
    }

    /** Returns true if any hub AprilTag is currently visible on either camera. */
    public boolean isHubVisible() {
        return getBestHubTarget().isPresent();
    }

    /**
     * Returns the AprilTag field layout for the 2026 Rebuilt game.
     * Used by FaceHubCommand for the odometry-based fallback bearing calculation.
     */
    public AprilTagFieldLayout getFieldLayout() {
        return fieldLayout;
    }

    // -----------------------------------------------------------------------
    // Private helpers
    // -----------------------------------------------------------------------

    private void updatePoseEstimates(PhotonCamera camera, PhotonPoseEstimator estimator) {
        for (var result : camera.getAllUnreadResults()) {
            // Try multi-tag coprocessor estimate first; fall back to lowest ambiguity single-tag
            Optional<EstimatedRobotPose> estimate = estimator.estimateCoprocMultiTagPose(result);
            if (estimate.isEmpty()) {
                estimate = estimator.estimateLowestAmbiguityPose(result);
            }
            estimate.ifPresent(est -> {
                int tagCount = est.targetsUsed.size();
                Matrix<N3, N1> stdDevs = (tagCount >= 2) ? MULTI_TAG_STD_DEVS : SINGLE_TAG_STD_DEVS;
                drivetrain.addVisionMeasurement(
                    est.estimatedPose.toPose2d(),
                    est.timestampSeconds,
                    stdDevs
                );
            });
        }
    }

    private int[] getAllianceHubTagIds() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return BLUE_HUB_TAG_IDS;
        }
        return RED_HUB_TAG_IDS; // default Red
    }

    private Optional<PhotonTrackedTarget> getBestTargetFromCamera(PhotonCamera camera, int[] validIds) {
        var results = camera.getAllUnreadResults();
        if (results.isEmpty()) return Optional.empty();
        var result = results.get(results.size() - 1); // most recent frame
        if (!result.hasTargets()) return Optional.empty();

        return result.getTargets().stream()
            .filter(t -> isInArray(t.getFiducialId(), validIds))
            .min((a, b) -> Double.compare(a.getPoseAmbiguity(), b.getPoseAmbiguity()));
    }

    private boolean isInArray(int id, int[] arr) {
        for (int v : arr) {
            if (v == id) return true;
        }
        return false;
    }
}
