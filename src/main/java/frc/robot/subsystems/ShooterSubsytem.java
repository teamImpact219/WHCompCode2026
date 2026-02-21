// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ShooterSubsytem extends SubsystemBase {
  // private final TalonFX shooter  = new TalonFX(10);
  private final TalonFX shooter  = new TalonFX(10);
   //private final CommandXboxController shootStick = new CommandXboxController(1);

   public void runShooter(){
    shooter.set(-.3);
   }

   public void stopShooter(){
    shooter.set(0);
   }



   //these commands dont work for some reason
  public Command runTalonCmd(){
    //return runOnce(() -> shooter.set(-.83));
     return runOnce(() -> shooter.set(-0.2));
  }

  public Command stopTalonCmd(){
    return runOnce(() -> shooter.set(0));
  }




  /** Creates a new ShooterSubsytem. */
  public ShooterSubsytem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}