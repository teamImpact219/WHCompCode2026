//base imports
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
//subsystem imports
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.UniCamVisionSubsystem;
//command imports
import com.ctre.phoenix6.swerve.SwerveRequest;
//util imports
import java.util.ArrayList;
//tag imports
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
public class uniCamTurnToTagCommand extends Command {
  //variables
    //vision variables
    UniCamVisionSubsystem vision;
    //drivetrain variables
    CommandSwerveDrivetrain drivetrain;
    SwerveRequest.FieldCentric driveCommand;
    //tag variables
    int id;
    double targetYaw = Math.PI/49.0;
    boolean foundTag = false;
  //constructor
  public uniCamTurnToTagCommand(
    int id, UniCamVisionSubsystem vision,
     CommandSwerveDrivetrain drivetrain, 
     SwerveRequest.FieldCentric driveCommand
     ) 
  {
    //reqs
    addRequirements(drivetrain);
    addRequirements(vision);
    //assignment
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.id = id;
    this.driveCommand = driveCommand;
  }

  @Override
  public void initialize() {}
  @Override
  public void execute() {
    
    
    ArrayList<PhotonPipelineResult> result = vision.getNewPiplineResults();
      for (int i = result.size()-1; i >= 0; i--){
        PhotonTrackedTarget target = vision.findTag(id, result.get(i));
        if (target != null){
          targetYaw = Math.toRadians(target.yaw);
          break;
        }
      }
      drivetrain.setControl(driveCommand.withRotationalRate(-2.5*targetYaw)); 
      //drivetrain.setControl(driveCommand.withRotationalRate(-1));
    }

  @Override
  public void end(boolean interrupted) {}
  @Override
  public boolean isFinished() {
    return 
    //tick > maxtick || 
    Math.abs(targetYaw) < Math.PI/50.0;
  }
}
