//base imports
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
//subsystem imports
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.UniCamVisionSubsystem;
//requester imports
import com.ctre.phoenix6.swerve.SwerveRequest;
//position imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;

//util imports
import java.util.Optional;
import java.lang.Math;
import edu.wpi.first.math.MathUtil;


/*
 * NOTE: RELIES ENTIRELY ON THE ABILITY TO DETERMINE CURRENT FIELD POS, 
 * PLEASE MAKE SURE TO EITHER MAINTAIN AN ACCURATE POSITION OR
 * COMPENSATE FOR THIS RISK
*/
public class fieldRelativeTurnToTagCommand extends Command {
  //variables
    //drivetrain variables
    CommandSwerveDrivetrain drivetrain;
    SwerveRequest.FieldCentric driveCommand;
    //tag variables
    int tagID;
    boolean tagExists = false;
    //position variables
    Pose2d botPosition;
    Pose2d tagPosition;
    double error;
  public fieldRelativeTurnToTagCommand(CommandSwerveDrivetrain drivetrain,SwerveRequest.FieldCentric driveCommand, int id) {
    //reqs
    this.drivetrain = drivetrain;
    this.driveCommand = driveCommand;
    tagID = id;
    //assignmnet
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    Optional<Pose3d> optPosContainer = UniCamVisionSubsystem.apriltagPositions.getTagPose(tagID);
    if (optPosContainer.isPresent()){
      tagPosition = optPosContainer.get().toPose2d();
      tagExists = true;
    }

  }
  @Override
  public void execute() {
    if (tagExists){
      //gets the current bot position
      botPosition = drivetrain.getState().Pose;
      //finds the error in the angles between the tag and bot positions (rad)
      error = 
      Math.atan2(
        tagPosition.getY()-botPosition.getY(),
        tagPosition.getX()-botPosition.getX()
      ) - botPosition.getRotation().getRadians();
      //normalizes to [-pi, pi]
      error = MathUtil.angleModulus(error);
      //actually rotating to it
      drivetrain.setControl(driveCommand.withRotationalRate(-0.5*error)); 
    }
  }
  @Override
  public void end(boolean interrupted) {drivetrain.setControl(driveCommand.withRotationalRate(0)); }
  
  @Override
  public boolean isFinished() {return !tagExists || Math.abs(error) < Math.PI/180.0;}

  //handles instantiation but also accounts for team color/side
  public static fieldRelativeTurnToTagCommand instantiateObject(
    CommandSwerveDrivetrain drivetrain,SwerveRequest.FieldCentric driveCommand,
    int id_red, int id_blue, int id_backup
  ){
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()){
      if (alliance.get() == DriverStation.Alliance.Red){
        return new fieldRelativeTurnToTagCommand(drivetrain, driveCommand, id_red);
      }
      //blue otherwise, theres no other teams
      else{
        return new fieldRelativeTurnToTagCommand(drivetrain, driveCommand, id_blue);
      }
    }
        return new fieldRelativeTurnToTagCommand(drivetrain, driveCommand, id_backup);
  }
}
