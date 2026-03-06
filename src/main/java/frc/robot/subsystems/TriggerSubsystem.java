// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TriggerSubsystem extends SubsystemBase {
   private final SparkMax trigger  = new SparkMax(15, MotorType.kBrushless);
   private final SparkMax agitator  = new SparkMax(18, MotorType.kBrushless); 




   public void runTrigger(){
   agitator.set(0.65);
    trigger.set(-1);
   }

   public void debugAgitator(){
    agitator.set(0.2);
   }

    public void stopDebugAgitator(){
    agitator.set(0);
   }

   public void stopTrigger(){
    trigger.set(0);
   agitator.set(0);
   }


   //these commands dont work for some reason
   public Command stopTriggerCmd(){
    return runOnce(() -> trigger.set(0));
   }
    public Command runTriggerCmd(){
    return runOnce(() -> trigger.set(-1));
    

   }

   public Command stopAgitatorCmd2(){
    return runOnce(() -> agitator.set(0));
   }
    public Command runAgitatorCmd2(){
    return runOnce(() -> agitator.set(.65));
    

   }
    public void runAgitatorCmd(){
    
     agitator.set(0.65);

   }

   public void stopAgitatorCmd(){
    
     agitator.set(0);

   }




  /** Creates a new ShooterSubsytem. */
  public TriggerSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}