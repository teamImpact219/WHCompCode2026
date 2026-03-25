// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.cameraserver.*;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GroundHarvesterSubsystem;
import frc.robot.subsystems.HarvesterSubsystem;
import frc.robot.subsystems.ShooterSubsytem;
import frc.robot.subsystems.TriggerSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.RobotController;


public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    //NEVERE EVER DO THIS THIS IS BAD PUT IN CONTAINER

     // private final ShooterSubsytem shooter = new ShooterSubsytem();
   // private final TriggerSubsystem trigger = new TriggerSubsystem();

    //private final HarvesterSubsystem harv = new HarvesterSubsystem();
    //private final HarvesterCommand harvest = new HarvesterCommand(harv);

   // private final GroundHarvesterSubsystem groundHarv = new GroundHarvesterSubsystem();
 

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    
    UsbCamera frontCam;
    UsbCamera insideCam;
    
        public Robot() {
        m_robotContainer = new RobotContainer();
        frontCam= CameraServer.startAutomaticCapture(0);
        insideCam= CameraServer.startAutomaticCapture(1);
        frontCam.setFPS(20);

        
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 

        SmartDashboard.putNumber("ShooterSpeed", m_robotContainer.shooter.getShooterRPM());
        SmartDashboard.putNumber("TriggerSpeed", m_robotContainer.trigger.getTriggerRPM());
        SmartDashboard.putNumber("AgitatorSpeed", m_robotContainer.trigger.getAgitatorRPM());
        SmartDashboard.putNumber("GroundHarvSpeed", m_robotContainer.groundHarv.getGroundHarvRPM());
        SmartDashboard.putNumber("DropDownHarvSpeed", m_robotContainer.harv.getDropHarvRPM());
        //So this voltage 
        SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());


        
//         SmartDashboard.putData("Swerve Drive", new Sendable() {
//   @Override
//   public void initSendable(SendableBuilder builder) {
//     builder.setSmartDashboardType("SwerveDrive");

//     builder.addDoubleProperty("Front Left Angle", () -> frontLeftModule.getAngle().getRadians(), null);
//     builder.addDoubleProperty("Front Left Velocity", () -> frontLeftModule.getVelocity(), null);

//     builder.addDoubleProperty("Front Right Angle", () -> frontRightModule.getAngle().getRadians(), null);
//     builder.addDoubleProperty("Front Right Velocity", () -> frontRightModule.getVelocity(), null);

//     builder.addDoubleProperty("Back Left Angle", () -> backLeftModule.getAngle().getRadians(), null);
//     builder.addDoubleProperty("Back Left Velocity", () -> backLeftModule.getVelocity(), null);

//     builder.addDoubleProperty("Back Right Angle", () -> backRightModule.getAngle().getRadians(), null);
//     builder.addDoubleProperty("Back Right Velocity", () -> backRightModule.getVelocity(), null);

//     builder.addDoubleProperty("Robot Angle", () -> getRotation().getRadians(), null);
//   }
// });


    }
    // after Auton when switching to teleop this will turn of things we specify such as the shooter ir the inexer 
    @Override
    public void disabledInit() {
        m_robotContainer.shooter.stopShooter();
        m_robotContainer.harv.stopDropHarvest();
        m_robotContainer.trigger.stopTrigger();
        m_robotContainer.trigger.stopAgitatorCmd2();
        m_robotContainer.groundHarv.stopGroundHarvestCmd();
        

    }

    @Override
    public void disabledPeriodic() {}
    //this can reboot setting when the robot leaves Auton (sensors) or at start match
    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
