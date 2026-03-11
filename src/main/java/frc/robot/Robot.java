// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.cameraserver.*;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.GroundHarvesterSubsystem;
import frc.robot.subsystems.HarvesterSubsystem;
import frc.robot.subsystems.ShooterSubsytem;
import frc.robot.subsystems.TriggerSubsystem;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

      private final ShooterSubsytem shooter = new ShooterSubsytem();
   // private final TriggerSubsystem trigger = new TriggerSubsystem();

    //private final HarvesterSubsystem harv = new HarvesterSubsystem();
    //private final HarvesterCommand harvest = new HarvesterCommand(harv);

    private final GroundHarvesterSubsystem groundHarv = new GroundHarvesterSubsystem();

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

        SmartDashboard.putNumber("ShooterSpeed", shooter.getShooterRPM());
        SmartDashboard.putNumber("TriggerSpeed", m_robotContainer.trigger.getIsTriggerOn());
        SmartDashboard.putNumber("AgitatorSpeed", m_robotContainer.trigger.getIsAgitatorOn());
        SmartDashboard.putNumber("GroundHarvSpeed", groundHarv.getIsGroundHarvOn());
        SmartDashboard.putNumber("DropDownHarvSpeed", m_robotContainer.harv.getIsHarvOn());
    }
    // after Auton when switching to teleop this will turn of things we specify such as the shooter ir the inexer 
    @Override
    public void disabledInit() {
        shooter.stopShooter();
        m_robotContainer.harv.stopDropHarvest();
        m_robotContainer.trigger.stopTrigger();
        m_robotContainer.trigger.stopAgitatorCmd();

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
