package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

import java.util.Optional;

/**
 * Rotates the robot to face the scoring hub while allowing the driver to
 * translate freely. Runs for as long as the bound button is held.
 *
 * <p>Strategy:
 * <ol>
 *   <li><b>Primary:</b> If a hub AprilTag is visible, add its yaw offset to the
 *       robot's current field heading to compute the bearing toward the tag.
 *   <li><b>Fallback:</b> If no tag is visible, compute the bearing from the
 *       robot's odometry pose to the known hub field position (averaged from the
 *       AprilTag field layout).
 * </ol>
 *
 * <p>Heading PID gains ({@link #HEADING_KP} etc.) should be tuned on the robot.
 */
public class FaceHubCommand extends Command {

    // TODO (BEFORE DEPLOYMENT): Tune these heading PID gains on the actual robot.
    //   - Hold driver Y and watch how the robot rotates toward the hub.
    //   - If rotation is sluggish or barely moves: increase HEADING_KP.
    //   - If the robot oscillates/overshoots and wobbles: decrease HEADING_KP.
    //   - HEADING_KI and HEADING_KD can usually stay at 0.0 to start.
    //   - Typical working range for kP on a swerve drive: 3.0 – 10.0.
    private static final double HEADING_KP = 7.0;
    private static final double HEADING_KI = 0.0;
    private static final double HEADING_KD = 0.0;

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final CommandXboxController driverController;
    private final double maxSpeed;

    private final SwerveRequest.FieldCentricFacingAngle driveToAngle;

    public FaceHubCommand(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem vision,
            CommandXboxController driverController,
            double maxSpeed,
            double maxAngularRate) {

        this.drivetrain = drivetrain;
        this.vision = vision;
        this.driverController = driverController;
        this.maxSpeed = maxSpeed;

        driveToAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(maxSpeed * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withHeadingPID(HEADING_KP, HEADING_KI, HEADING_KD);
        driveToAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // Apply the same cube-law scaling used by the default drive command.
        double leftY = driverController.getLeftY();
        double leftX = driverController.getLeftX();
        double velX = -leftY * Math.pow(Math.abs(leftY), 2) * maxSpeed;
        double velY = -leftX * Math.pow(Math.abs(leftX), 2) * maxSpeed;

        Rotation2d targetHeading = computeTargetHeading();

        drivetrain.setControl(
            driveToAngle
                .withVelocityX(velX)
                .withVelocityY(velY)
                .withTargetDirection(targetHeading)
        );
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until the button is released
    }

    // -----------------------------------------------------------------------
    // Heading calculation helpers
    // -----------------------------------------------------------------------

    private Rotation2d computeTargetHeading() {
        // Primary: direct AprilTag yaw
        var targetOpt = vision.getBestHubTarget();
        if (targetOpt.isPresent()) {
            // PhotonVision yaw: positive = target is to the LEFT of camera center.
            // Adding the yaw to the current heading rotates the robot toward the tag.
            double yawDeg = targetOpt.get().getYaw();
            Rotation2d currentHeading = drivetrain.getState().Pose.getRotation();
            return currentHeading.plus(Rotation2d.fromDegrees(yawDeg));
        }

        // Fallback: compute bearing from odometry pose to known hub field position
        return computePoseFallbackHeading();
    }

    private Rotation2d computePoseFallbackHeading() {
        int[] hubIds = getAllianceHubTagIds();
        var layout = vision.getFieldLayout();

        // Average the X/Y positions of both hub tags to find the hub face center
        double sumX = 0.0, sumY = 0.0;
        int count = 0;
        for (int id : hubIds) {
            var tagPose = layout.getTagPose(id);
            if (tagPose.isPresent()) {
                sumX += tagPose.get().getX();
                sumY += tagPose.get().getY();
                count++;
            }
        }

        if (count == 0) {
            // No layout data available — hold current heading
            return drivetrain.getState().Pose.getRotation();
        }

        double hubX = sumX / count;
        double hubY = sumY / count;

        var robotPose = drivetrain.getState().Pose;
        double dx = hubX - robotPose.getX();
        double dy = hubY - robotPose.getY();

        return new Rotation2d(Math.atan2(dy, dx));
    }

    private int[] getAllianceHubTagIds() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return VisionSubsystem.BLUE_HUB_TAG_IDS;
        }
        return VisionSubsystem.RED_HUB_TAG_IDS;
    }
}
