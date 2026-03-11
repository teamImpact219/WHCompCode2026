package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundHarvesterSubsystem extends SubsystemBase {
  private final XboxController driveStick = new XboxController(0);
  private final TalonFX groundHarvest = new TalonFX(12);
  // private final TriggerSubsystem trigger = new TriggerSubsystem();
  // private double isAgitatorOn;
  public double isGroundHarvOn = 0;

  
  
  public void runGroundHarvest (){
    groundHarvest.set(-0.3);
    isGroundHarvOn = .3;
  }

  public void stopHarvest(){
    groundHarvest.set(0);
    isGroundHarvOn = 0;
  }

  public Command runGroundHarvestCmd(){
    return runOnce(() -> runGroundHarvest());
  };

  public Command stopGroundHarvestCmd(){
    return runOnce(() -> stopHarvest());
  };

  public double getIsGroundHarvOn(){
    return isGroundHarvOn;
  }



  /** Creates a new ShooterSubsytem. */
  public GroundHarvesterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

