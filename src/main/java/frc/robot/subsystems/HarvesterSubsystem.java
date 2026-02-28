// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import javax.swing.*;

public class HarvesterSubsystem extends SubsystemBase {
  private final XboxController driveStick = new XboxController(0);
  private final SparkMax runDropHarvester  = new SparkMax(16, MotorType.kBrushless);
  //private final SparkMax dropHarvest = new SparkMax(17, MotorType.kBrushless);
  private final TalonFX dropHarvest  = new TalonFX(18); //ID THIS
  private final DigitalInput topSwitch = new DigitalInput(0);
  private final DigitalInput botSwitch = new DigitalInput(1);


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

  public Command dropHarvestCMD(){
    return runOnce(() -> lowerDropHarvest());
  };

  public void raiseDropHarvest(){
    double startTime = System.currentTimeMillis();

  

    //dropHarvest.set(.2);
    System.out.println(topSwitch.get());
    
      if(!topSwitch.get()){
        dropHarvest.set(0);

      }
     

      while(topSwitch.get() || (System.currentTimeMillis()- startTime) < 1500){
        dropHarvest.set(-.25);
      }
      if(topSwitch.get() || (System.currentTimeMillis()- startTime) < 1500){
        dropHarvest.set(0);
      }
    
  }

  public void lowerDropHarvest(){
    double startTime = System.currentTimeMillis();
    System.out.println(!botSwitch.get());

    //dropHarvest.set(.2);

    if(!botSwitch.get()){
        dropHarvest.set(0);

      }
      while(botSwitch.get() || (System.currentTimeMillis()- startTime) < 1500){
        dropHarvest.set(.25);
        
      }

      if(!botSwitch.get()|| (System.currentTimeMillis()- startTime) < 1500){
        dropHarvest.set(0);

      }

  }



  /** Creates a new ShooterSubsytem. */
  public HarvesterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

