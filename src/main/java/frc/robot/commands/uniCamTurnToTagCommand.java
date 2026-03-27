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
import java.util.Optional;
//team imports
import edu.wpi.first.wpilibj.DriverStation;

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
    double targetYaw = 0;
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
      for (int i = result.size()-1; i >= 0; i--){
        PhotonTrackedTarget target = vision.findTag(id, result.get(i));
        if (target != null){
          foundTag = true;
          targetYaw = Math.toRadians(target.yaw);
          break;
        }
      }
      drivetrain.setControl(driveCommand.withRotationalRate(-0.5*targetYaw)); 
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

  //handles instantiation but also accounts for team color/side
  public static uniCamTurnToTagCommand instantiateObject(
    int id_red, int id_blue, int id_backup, UniCamVisionSubsystem vision,
    CommandSwerveDrivetrain drivetrain, 
    SwerveRequest.FieldCentric driveCommand
  ){
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()){
      if (alliance.get() == DriverStation.Alliance.Red){
        return new uniCamTurnToTagCommand(id_red, vision, drivetrain, driveCommand);
      }
      //blue otherwise, theres no other teams
      else{
        return new uniCamTurnToTagCommand(id_blue, vision, drivetrain, driveCommand);
      }
    }
        return new uniCamTurnToTagCommand(id_backup, vision, drivetrain, driveCommand);
  }
}
