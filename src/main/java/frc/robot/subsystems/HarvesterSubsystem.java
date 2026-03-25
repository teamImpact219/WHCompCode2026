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
  private final SparkMax runDropHarvester  = new SparkMax(16, MotorType.kBrushless);
  //private final SparkMax dropHarvest = new SparkMax(17, MotorType.kBrushless);
  private final TalonFX dropHarvest  = new TalonFX(17);
  private double isHarvOn =0;
   //negative in up
  //positive is down

  private final DigitalInput topSwitch = new DigitalInput(0);
  private final DigitalInput botSwitch = new DigitalInput(1);


  public void runDropHarvest(){
    runDropHarvester.set(-1);
    isHarvOn = 1;
  }
  
  public void stopDropHarvest(){
    runDropHarvester.set(0);
    isHarvOn = 0;
  }

  public double getDropHarvRPM() {
    return runDropHarvester.getEncoder().getVelocity();
   }



   public void stopMoving(){
    dropHarvest.set(0);
    runDropHarvester.set(0);
  }






  public Command dropHarvestCMD(){
    return runOnce( () -> lowerHarvesterCmd());
  };

  public Command raiseHarvestCmd(){
    return runOnce(() -> raiseForDumpCmd());
  };

  public Command runHarvCmd(){
    return runOnce(() -> runDropHarvest());
  };


  public Command stopHarvCmd(){
    return runOnce(() -> stopDropHarvest());
  };



  //negative is down and postive is up for harvester moving

//pushed false
//unpushed true
 
  public Command raiseHarvesterCmd() {
    return new Command() {
      private long startTime;

      @Override
      public void initialize() {
        startTime = System.currentTimeMillis();
      }

      @Override
      public void execute() {
        if (topSwitch.get()) {

          dropHarvest.set(.15);

        }
        else
        {
          dropHarvest.set(0);
        }       
      }

      @Override
      public boolean isFinished() {
        return !topSwitch.get() || (System.currentTimeMillis() - startTime) > 3000;
      }

      @Override
      public void end(boolean interupt) {
        dropHarvest.set(0);
      }

      @Override
      public java.util.Set<edu.wpi.first.wpilibj2.command.Subsystem> getRequirements() {
        return java.util.Set.of(HarvesterSubsystem.this);
      }

    };

  }

    public Command lowerHarvesterCmd() {
      return new Command() {
        private long startTime;

        @Override
        public void initialize() {
          startTime = System.currentTimeMillis();
        }

        @Override
        public void execute() {
          if (botSwitch.get()) {

            dropHarvest.set(-.3);

          } else {
            dropHarvest.set(0);
          }

        }

        @Override
        public boolean isFinished() {
          return !botSwitch.get() || (System.currentTimeMillis() - startTime) > 2200;
        }

        @Override
        public void end(boolean interupt) {
          dropHarvest.set(0);
        }

        @Override
        public java.util.Set<edu.wpi.first.wpilibj2.command.Subsystem> getRequirements() {
          return java.util.Set.of(HarvesterSubsystem.this);
        }

      };

    }

    public Command raiseForDumpCmd() {
      return new Command() {
        private long startTime;

        @Override
        public void initialize() {
          startTime = System.currentTimeMillis();
        }

        @Override
        public void execute() {
          if (topSwitch.get()) {

            dropHarvest.set(.3);
            runDropHarvester.set(-1);

          } else {
            dropHarvest.set(0);
            runDropHarvester.set(0);
          }

        }

        @Override
        public boolean isFinished() {
          return !topSwitch.get() || (System.currentTimeMillis() - startTime) > 1400;
        }

        @Override
        public void end(boolean interupt) {
          dropHarvest.set(0);
          runDropHarvester.set(0);
        }

        @Override
        public java.util.Set<edu.wpi.first.wpilibj2.command.Subsystem> getRequirements() {
          return java.util.Set.of(HarvesterSubsystem.this);
        }

      };

    }

  /** Creates a new ShooterSubsytem. */
  public HarvesterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

