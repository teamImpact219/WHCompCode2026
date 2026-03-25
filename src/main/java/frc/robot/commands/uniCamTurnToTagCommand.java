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
    int[] ids;
    double targetYaw = 0;
    boolean foundTag = false;
  //constructor
  public uniCamTurnToTagCommand(
    int[] ids, UniCamVisionSubsystem vision,
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
    this.ids = ids;
    this.driveCommand = driveCommand;
  }

  //NOTE: THE SAME COMMAND OBJECT CAN BE REUSED, YOU JUST NEED TO RESET THE PREVIOUSLY USED VARS 
  // THINGS LIKE "targetYaw" WILL CARRY OVER FROM PREVIOUS USES AND MESS WITH COMMAND BEHAVIORS 
  @Override
  public void initialize() {
    targetYaw = 0.0;

  }
  @Override
  public void execute() {
    foundTag = false;
    ArrayList<PhotonPipelineResult> result = vision.getNewPiplineResults();
      outer:
      for (int i = result.size()-1; i >= 0; i--){
        for (int id : ids) {
          PhotonTrackedTarget target = vision.findTag(id, result.get(i));
          if (target != null){
            foundTag = true;
            targetYaw = Math.toRadians(target.yaw);
            break outer;
          }
        }
      }
      drivetrain.setControl(driveCommand.withVelocityX(0).withVelocityY(0).withRotationalRate(-0.5*targetYaw));
      //drivetrain.setControl(driveCommand.withRotationalRate(-1));
    }

  @Override
  public void end(boolean interrupted) {}
  @Override
  public boolean isFinished() {
    return 
    foundTag &&
    Math.abs(targetYaw) < Math.PI/50.0;
  }
}
