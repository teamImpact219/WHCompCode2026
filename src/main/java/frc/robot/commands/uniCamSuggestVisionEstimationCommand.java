//base imports
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
//subsystem imports
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.UniCamVisionSubsystem;
//april tag imports
import org.photonvision.targeting.PhotonPipelineResult;
//util imports
import java.util.ArrayList;
import java.util.Optional;
//pose imports 
import org.photonvision.EstimatedRobotPose;

public class uniCamSuggestVisionEstimationCommand extends Command {
  //variables
    //vision 
    UniCamVisionSubsystem vision;
    //drivetrain
    CommandSwerveDrivetrain drivetrain;

  public uniCamSuggestVisionEstimationCommand(CommandSwerveDrivetrain drivetrain, UniCamVisionSubsystem vision) {
    //reqs
    addRequirements(vision);
    //assignment
    this.drivetrain = drivetrain;
    this.vision = vision;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
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

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {return false;}
  

}
