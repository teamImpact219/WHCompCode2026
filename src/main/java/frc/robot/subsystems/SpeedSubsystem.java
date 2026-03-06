// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SpeedSubsystem extends SubsystemBase {
  /** Creates a new SpeedSubsystem. */
  public SpeedSubsystem() {

    
  }
  double speed =.5;

  public void setSlowSpeed(){
                speed=.25;
        }
        public void setFastSpeed(){
                speed=.5;
        }

         public double getSpeed(){
                return speed;
        }
       


        public Command fastCmd(){
               return  runOnce(() -> setFastSpeed());
         }

         public Command slowCmd(){
               return  runOnce(() -> setSlowSpeed());
         }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
