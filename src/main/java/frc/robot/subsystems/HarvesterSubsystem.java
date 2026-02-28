// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

public class HarvesterSubsystem extends SubsystemBase {
  private final SparkMax runDropHarvester  = new SparkMax(16, MotorType.kBrushless);
  private final TalonFX dropHarvest  = new TalonFX(18); //ID THIS
  private final DigitalInput topSwitch = new DigitalInput(0);
  private final DigitalInput botSwitch = new DigitalInput(1);

  public void runDropHarvest(){
    runDropHarvester.set(1);
  }

  public void stopDropHarvest(){
    runDropHarvester.set(0);
  }

  public Command dropHarvestCMD(){
    return lowerDropHarvestCmd();
  }

  public Command raiseDropHarvestCmd() {
    return run(() -> dropHarvest.set(-0.25))
        .until(() -> !topSwitch.get())
        .withTimeout(1.5)
        .finallyDo(() -> dropHarvest.set(0));
  }

  public Command lowerDropHarvestCmd() {
    return run(() -> dropHarvest.set(0.25))
        .until(() -> !botSwitch.get())
        .withTimeout(1.5)
        .finallyDo(() -> dropHarvest.set(0));
  }

  /** Creates a new HarvesterSubsystem. */
  public HarvesterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
