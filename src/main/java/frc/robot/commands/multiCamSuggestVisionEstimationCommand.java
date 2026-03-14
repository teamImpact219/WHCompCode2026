//base imports
package frc.robot.commands;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.Command;
//subsystem imports
import frc.robot.subsystems.UniCamVisionSubsystem;
import frc.robot.subsystems.MultiCamVisionSubsytem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class multiCamSuggestVisionEstimationCommand extends Command {
  //variables
    //subsystems
    MultiCamVisionSubsytem multiVision;
    //drivetrain
    CommandSwerveDrivetrain drivetrain;


  public multiCamSuggestVisionEstimationCommand(CommandSwerveDrivetrain drivetrain, MultiCamVisionSubsytem multiVisionSubsystem) {
    //reqs
    addRequirements(multiVisionSubsystem);
    addRequirements(drivetrain);
    //assignmmnet
    multiVision = multiVisionSubsystem;
    this.drivetrain = drivetrain;
  }

  @Override
  public void initialize() {
    for (UniCamVisionSubsystem vision: multiVision.getVisionSubsystems()){
      //literally the pose estimation suggestion code from the uni cam stuff lol
      ArrayList<PhotonPipelineResult> result = vision.getNewPiplineResults();
      for (int i = result.size()-1; i >= 0; i--){
        Optional<EstimatedRobotPose> pose = vision.getFieldPoseEstimate(result.get(i));
        if (!pose.isEmpty()){
          drivetrain.addVisionMeasurement(
            pose.get().estimatedPose.toPose2d(),
            pose.get().timestampSeconds
          );
          break;
        }
      }
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {return false;}
}
