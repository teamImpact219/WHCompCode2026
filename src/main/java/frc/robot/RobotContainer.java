// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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

import frc.robot.commands.uniCamSuggestVisionEstimationCommand;
import frc.robot.commands.uniCamTurnToTagCommand;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants3;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GroundHarvesterSubsystem;
import frc.robot.subsystems.HarvesterSubsystem;
import frc.robot.subsystems.ShooterSubsytem;

import frc.robot.subsystems.TriggerSubsystem;
import frc.robot.subsystems.UniCamVisionSubsystem;
import edu.wpi.first.wpilibj2.command.Subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;





public class RobotContainer extends SubsystemBase{

    public final CommandSwerveDrivetrain drivetrain = TunerConstants3.createDrivetrain();
    private final double BASE_SPEED = 0.5 * TunerConstants3.kSpeedAt12Volts.in(MetersPerSecond);
    private final double[] MaxSpeed = { BASE_SPEED };
    
    // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); //was .75 now .5 2-28-26        3/4 of a rotation per second
                                                                                      // max angular velocity
    // not swerev variables

    private final CommandXboxController codriverController = new CommandXboxController(1);

    public final ShooterSubsytem shooter = new ShooterSubsytem();
    public final TriggerSubsystem trigger = new TriggerSubsystem();

    public final HarvesterSubsystem harv = new HarvesterSubsystem();
    //private final HarvesterCommand harvest = new HarvesterCommand(harv);

    public final GroundHarvesterSubsystem groundHarv = new GroundHarvesterSubsystem();
    // private final GroundHarvesterCommand groundHarvest = new
    // GroundHarvesterCommand(groundHarv);

    

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(BASE_SPEED * 0.2).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 20% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(BASE_SPEED);

    private final CommandXboxController driverController = new CommandXboxController(0);
    //VISION STUFF
    public final UniCamVisionSubsystem frontVision = new UniCamVisionSubsystem(VisionConstants.kFrontCamName, VisionConstants.kFrontRobotToCam);
    public final UniCamVisionSubsystem sideVision  = new UniCamVisionSubsystem(VisionConstants.kSideCamName,  VisionConstants.kSideRobotToCam);
    public final uniCamTurnToTagCommand turnToCommand = new uniCamTurnToTagCommand(new int[]{9, 10, 25, 26}, frontVision, drivetrain, drive);
    public final uniCamSuggestVisionEstimationCommand frontVisionEstimation = new uniCamSuggestVisionEstimationCommand(drivetrain, frontVision);
    public final uniCamSuggestVisionEstimationCommand sideVisionEstimation  = new uniCamSuggestVisionEstimationCommand(drivetrain, sideVision);




    //private final SendableChooser<Command> autoChooser;

    public RobotContainer() {



        //autoChooser = AutoBuilder.buildAutoChooser("ShootTest");
        //SmartDashboard.putData("Auto Mode", autoChooser);


        NamedCommands.registerCommand("CloseShot",  shooter.runTalonCmd());

        NamedCommands.registerCommand("runTrigger", trigger.runTriggerCmd());

        NamedCommands.registerCommand("stopShooter", shooter.stopTalonCmd());
         
        NamedCommands.registerCommand("Dropharvest", harv.lowerHarvesterCmd());

        NamedCommands.registerCommand("RaiseHarvest", harv.raiseForDumpCmd());

        NamedCommands.registerCommand("LongShot", shooter.runLongShotCmd());

        NamedCommands.registerCommand("stopTrigger", trigger.stopTriggerCmd());

        NamedCommands.registerCommand("runGroundHarvest", groundHarv.runGroundHarvestCmd());

        NamedCommands.registerCommand("runAgitator", trigger.runAgitatorCmd2());

        NamedCommands.registerCommand("stopAgitator", trigger.stopAgitatorCmd2());

        NamedCommands.registerCommand("stopGroundHarvest", groundHarv.stopGroundHarvestCmd());
        
        NamedCommands.registerCommand("runHarv", harv.runHarvCmd());

        NamedCommands.registerCommand("stopHarv", harv.stopHarvCmd());



       

        drivetrain.configurePathPlanner();
        
        configureBindings();


        //widgets for smartDashboard 
        // SmartDashboard.putNumber("ShooterSpeed", shooter.getShooterRPM());
        // SmartDashboard.putNumber("TriggerSpeed", trigger.getIsTriggerOn());
        // SmartDashboard.putNumber("AgitatorSpeed", trigger.getIsAgitatorOn());
        // SmartDashboard.putNumber("GroundHarvSpeed", groundHarv.getIsGroundHarvOn());
        // SmartDashboard.putNumber("DropDownHarvSpeed", harv.getIsHarvOn());
    }

    private void configureBindings() {
        //vision stuff
                //constantly contributes position estimations from both cameras
                frontVision.setDefaultCommand(frontVisionEstimation);
                sideVision.setDefaultCommand(sideVisionEstimation);
                bindVisionController();
        // buttons we made
        bindJoystickX();
        bindJoysticky();  
        bindJoystickA();
        bindJoystickb();
        bindRightBumper();
        bindLeftBumper();
        bindLeftDPad();
        bindDownDPad();


        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
               // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * Math.pow((driverController.getLeftY()),2)* MaxSpeed[0])    
                        .withVelocityY(-driverController.getLeftX()* Math.pow((driverController.getLeftX()),2)* MaxSpeed[0]) // Drive left with negative X (left)
                        .withRotationalRate(-driverController.getRightX() * Math.pow((driverController.getRightX()),2)* MaxAngularRate) // Drive counterclockwise
                                                                                            // with negative X (left)
                ));
                // drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * Math.pow((driverController.getLeftY()),3)* MaxSpeed)    
                //         .withVelocityY( Math.pow((driverController.getLeftX()),3)* MaxSpeed) // Drive left with negative X (left)
                //         .withRotationalRate(-driverController.getRightX() * Math.pow((driverController.getRightX()),3)* MaxAngularRate) // Drive counterclockwise
                //                                                                             // with negative X (left)
                // ));

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



    // START OF THE BUTTON PRESSES

    
     private void bindLeftDPad() {
        // Left D-pad: boost to 2x base speed
        driverController.povLeft().onTrue(
            Commands.runOnce(() -> {
                MaxSpeed[0] = BASE_SPEED * 1.5;
                SmartDashboard.putNumber("MaxSpeed", MaxSpeed[0]);
                System.out.println("LEFT D-PAD PRESSED — BOOST ACTIVATED");
                System.out.println("Base Speed:    " + BASE_SPEED + " m/s");
                System.out.println("Current Speed: " + MaxSpeed[0] + " m/s  (2x)");
            })
        );
        // codriverController.povLeft().toggleOnTrue((harv.startEnd(
        // () -> harv.raiseHarvestCmd(),
        // () -> harv.stopMoving())));


        codriverController.povLeft().onTrue(harv.raiseHarvesterCmd());

    }
 
    private void bindDownDPad() {
        // Down D-pad: reset back to base speed
        driverController.povDown().onTrue(
            Commands.runOnce(() -> {
                MaxSpeed[0] = BASE_SPEED;
                SmartDashboard.putNumber("MaxSpeed", MaxSpeed[0]);
                System.out.println("DOWN D-PAD PRESSED — SPEED RESET");
                System.out.println("Current Speed: " + MaxSpeed[0] + " m/s  (back to base)");
            })
        );
    }




//     System.out.println("fast");
//         }
    //added by Max; for vision control
    private void bindVisionController(){
        driverController.b().whileTrue(turnToCommand);


    }

    private void bindJoystickA() {

        // runs the trigger (bottom row of wheels on the shooter)
        codriverController.a().toggleOnTrue(
                trigger.startEnd(
                        () -> trigger.runTrigger(), // Start action
                        () -> trigger.stopTrigger() // End action
                ));

        // toggle ground harvest on and off with the press of a button
        // driverController.a().toggleOnTrue(
        //         harv.startEnd(
        //                 () -> harv.runDropHarvest(),
        //                 () -> harv.stopDropHarvest()));
        
        // driverController.a().toggleOnTrue(
        //         groundHarv.startEnd(
        //                 () -> groundHarv.runGroundHarvest(), // Start action
        //                 () -> groundHarv.stopHarvest() // End action
        //         ));

        // driverController.a().toggleOnTrue(
        //         trigger.startEnd(
        //                 () -> trigger.runAgitatorCmd(), // Start action
        //                 () -> trigger.stopAgitatorCmd() // End action
        //         ));

        driverController.a().toggleOnTrue(
    Commands.parallel(
        harv.startEnd(
            () -> harv.runDropHarvest(),
            () -> harv.stopDropHarvest()),
        groundHarv.startEnd(
            () -> groundHarv.runGroundHarvest(),
            () -> groundHarv.stopHarvest()),
        trigger.startEnd(
            () -> trigger.runAgitatorCmd(),
            () -> trigger.stopAgitatorCmd())
    )
);
    }

     private void bindJoysticky() {
        // codriverController.y().toggleOnTrue((harv.startEnd(
        // () -> harv.raiseForDump(),
        // () -> harv.stopMoving())));


        codriverController.y().onTrue(harv.raiseForDumpCmd());

    }

    private void bindJoystickX() {

        // shoots the shooter
        codriverController.x().toggleOnTrue(
                shooter.startEnd(
                        () -> shooter.runCloseShooter(), // Start action
                        () -> shooter.stopShooter() // End action
                ));

     }


    //THIS IS FOR DEBUGGING THE AGITATOR UNTILL IT WORKS AS INTENED (MECHANICAL PROBLEM)
     private void bindJoystickb() {


        // codriverController.b().toggleOnTrue((harv.startEnd(
        // () -> harv.lowerDropHarvest(),
        // () -> harv.stopMoving())));



        codriverController.b().onTrue(harv.lowerHarvesterCmd());
    }


        public void bindRightBumper(){
                codriverController.rightBumper().toggleOnTrue(shooter.startEnd( ()->
                shooter.runShooter(),
                () -> shooter.stopShooter()));

        }

        public void bindLeftBumper(){
                codriverController.leftBumper().toggleOnTrue(shooter.startEnd( ()->
                shooter.runCloserShooter(),
                () -> shooter.stopShooter()));

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
        //return new PathPlannerAuto("Straight Auto");
        // return new PathPlannerAuto("shootTest");
        //return shooter.runTalonCmd();
        return new PathPlannerAuto("rightPlayer");
       
    }

}       
