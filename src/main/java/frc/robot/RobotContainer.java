// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
//import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.*;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GroundHarvesterSubsystem;
import frc.robot.subsystems.HarvesterSubsystem;
import frc.robot.subsystems.ShooterSubsytem;
import frc.robot.subsystems.TriggerSubsystem;


public class RobotContainer {
    private double MaxSpeed = .5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                       // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity
    // not swerev variables
    // private final CommandXboxController driveStick= new CommandXboxController(1);
    private final CommandXboxController codriverController = new CommandXboxController(1);

    private final ShooterSubsytem shooter = new ShooterSubsytem();
    private final TriggerSubsystem trigger = new TriggerSubsystem();

    private final HarvesterSubsystem harv = new HarvesterSubsystem();
    //private final HarvesterCommand harvest = new HarvesterCommand(harv);

    private final GroundHarvesterSubsystem groundHarv = new GroundHarvesterSubsystem();
    // private final GroundHarvesterCommand groundHarvest = new
    // GroundHarvesterCommand(groundHarv);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        //autoChooser = AutoBuilder.buildAutoChooser("ShootTest");
        //SmartDashboard.putData("Auto Mode", autoChooser);


        NamedCommands.registerCommand("CloseShot",  shooter.runTalonCmd());

        NamedCommands.registerCommand("runTrigger", trigger.runTriggerCmd());

        NamedCommands.registerCommand("stopShooter", shooter.stopTalonCmd());
         
        NamedCommands.registerCommand("DropHarvester", harv.dropHarvestCMD());

        NamedCommands.registerCommand("LongShot", shooter.runLongShotCmd());

        NamedCommands.registerCommand("stopTrigger", trigger.stopTriggerCmd());

        NamedCommands.registerCommand("runGroundHarvest", groundHarv.runGroundHarvestCmd());


       

        drivetrain.configurePathPlanner();
        
        configureBindings();
    }

    private void configureBindings() {
        // buttons we made
        bindJoystickX();
        bindJoysticky();
        bindJoystickA();
        bindJoystickb();

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * Math.sqrt(Math.abs(driverController.getLeftY()))* MaxSpeed)    
                        .withVelocityY(-driverController.getLeftX()* Math.sqrt(Math.abs(driverController.getLeftX())) * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driverController.getRightX() * Math.sqrt(Math.abs(driverController.getRightX())) * MaxAngularRate) // Drive counterclockwise
                                                                                            // with negative X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // driverController.y().whileTrue(drivetrain.applyRequest(() -> brake));
        // driverController.b().whileTrue(drivetrain.applyRequest(() ->
        // point.withModuleDirection(new Rotation2d(-driverController.getLeftY(),
        // -driverController.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void bindJoystickA() {

        // runs the trigger (bottom row of wheels on the shooter)
        codriverController.a().toggleOnTrue(
                trigger.startEnd(
                        () -> trigger.runTrigger(), // Start action
                        () -> trigger.stopTrigger() // End action
                ));

        // toggle ground harvest on and off with the press of a button
        driverController.a().toggleOnTrue(

                groundHarv.startEnd(
                        () -> groundHarv.runGroundHarvest(),
                        () -> groundHarv.stopHarvest()));
    }

     private void bindJoysticky() {

        //runs the trigger (bottom row of wheels on the shooter)
        // driverController.y().onTrue(
        //         harv.runOnce(
        //              ()->  harv.raiseDropHarvest() // End action
        //  ) );
    }

    private void bindJoystickX() {

        // shoots the shooter
        codriverController.x().toggleOnTrue(
                shooter.startEnd(
                        () -> shooter.runShooter(), // Start action
                        () -> shooter.stopShooter() // End action
                ));

        // runs the drop down harvester
        driverController.x().toggleOnTrue(
                harv.startEnd(
                        () -> harv.runDropHarvest(), // Start action
                        () -> harv.stopDropHarvest() // End action
                ));

    }
    //THIS IS FOR DEBUGGING THE AGITATOR UNTILL IT WORKS AS INTENED (MECHANICAL PROBLEM)
     private void bindJoystickb() {

        codriverController.b().toggleOnTrue((trigger.startEnd(
        () -> trigger.debugAgitator(),
        () -> trigger.stopDebugAgitator())));
       
        // driverController.b().onTrue(
        //         harv.runOnce(
               
        //              ()->  harv.lowerDropHarvest() // End action
        //  ) );

    }

    public Command getAutonomousCommand() {
        // // Simple drive forward auton
        // final var idle = new SwerveRequest.Idle();
        // return Commands.sequence(
        //     // Reset our field centric heading to match the robot
        //     // facing away from our alliance station wall (0 deg).
        //     drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        //     // Then slowly drive forward (away from us) for 5 seconds.
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(0.5)
        //             .withVelocityY(0)
        //             .withRotationalRate(0)
        //     )
        //     .withTimeout(5.0),
        //     // Finally idle for the rest of auton
        //     drivetrain.applyRequest(() -> idle)
        // );
        
        // return new PathPlannerAuto("SlowTestAuto");
        // return new PathPlannerAuto("RightAuto");
        return new PathPlannerAuto("Straight Auto");
        // return new PathPlannerAuto("shootTest");
        //return shooter.runTalonCmd();
       
    }
}
