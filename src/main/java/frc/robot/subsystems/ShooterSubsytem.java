// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.ctre.phoenix6.StatusSignal;

public class ShooterSubsytem extends SubsystemBase {
  // private final TalonFX shooter  = new TalonFX(10);
  private final TalonFX shooter  = new TalonFX(10);
  // 
  
  
   //private final CommandXboxController shootStick = new CommandXboxController(1);
   

   private  double isShooterOn = 0;

   
   public void runShooter(){
    shooter.set(-1);
    isShooterOn=1;
    
   }

   public void runCloseShooter(){
    shooter.set(-0.83);
    isShooterOn=.83;
   }

   public void runCloserShooter(){
    shooter.set(-0.68);
    isShooterOn=0-.68;
   }




   //about 9ft 5 inch for 0.83 right on the money



   public void stopShooter(){
    shooter.set(0);
    isShooterOn=0;
   }

   public double getShooterRPM() {
    return shooter.getVelocity().getValueAsDouble()*60;
   }



   //these commands dont work for some reason
  public Command runTalonCmd(){
    return runOnce(() -> shooter.set(-0.83));
    
  }

  public Command runLongShotCmd(){
    return runOnce(() -> shooter.set(-1));
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

  public double getIsShooterOn(){
    return isShooterOn;
  }
}