// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HarvesterSubsystem extends SubsystemBase {
  private final XboxController driveStick = new XboxController(0);
  private final SparkMax runDropHarvester  = new SparkMax(16, MotorType.kBrushless);
  private final SparkMax dropharvest = new SparkMax(17, MotorType.kBrushless);


  //make it a hold until it reaches the limit switch. 

  // public void raiseHarvester (){
  //   dropharvest.set(0.3);
  // }

  // public void dropHarvester (){
  //   dropharvest.set(-0.3); //negative is counter-clockwise = drop harvester
  // }

  public void runDropHarvest(){
    runDropHarvester.set(1);
  }
  
  public void stopDropHarvest(){
    runDropHarvester.set(0);
  }



  /** Creates a new ShooterSubsytem. */
  public HarvesterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

