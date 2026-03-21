//base imports
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
//subsystem imports
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.UniCamVisionSubsystem;
//requester imports
import com.ctre.phoenix6.swerve.SwerveRequest;

//WIP
public class fieldRelativeTurnToTagCommand extends Command {
  //variables
    //drivetrain variables
    CommandSwerveDrivetrain drivetrain;
    SwerveRequest.FieldCentric driveCommand;
    //tag variables
    int tagID;
  public fieldRelativeTurnToTagCommand(CommandSwerveDrivetrain drivetrain,SwerveRequest.FieldCentric driveCommand, int id) {
    //reqs
    this.drivetrain = drivetrain;
    this.driveCommand = driveCommand;
    tagID = id;
    //assignmnet
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}
  @Override
  public void execute() {}
  @Override
  public void end(boolean interrupted) {}
  @Override
  public boolean isFinished() {return false;}
}
