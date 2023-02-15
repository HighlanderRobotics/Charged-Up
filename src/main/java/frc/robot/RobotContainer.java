// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.RoutingSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;


import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  // private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  // private ArmSubsystem armSubsystem = new ArmSubsystem();
  // private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  // private RoutingSubsystem routingSubsystem = new RoutingSubsystem();
  // private GrabberSubsystem grabberSubsystem = new GrabberSubsystem();
  private LEDSubsystem ledSubsystem = new LEDSubsystem();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController controller =
      new CommandXboxController(OperatorConstants.driverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Set default commands here
    swerveSubsystem.setDefaultCommand(swerveSubsystem.driveCommand(
      () -> -(Math.abs(Math.pow(controller.getLeftY(), 2)) + 0.05) * Math.signum(controller.getLeftY()), 
      () -> -(Math.abs(Math.pow(controller.getLeftX(), 2)) + 0.05) * Math.signum(controller.getLeftX()), 
      () -> -(Math.abs(Math.pow(controller.getRightX(), 2)) + 0.05) * Math.signum(controller.getRightX()), 
      true, 
      true));
    // this is a little sus, might have to change logic to use subsystems separately or combine routing and intake subsystem
    // elevatorSubsystem.setDefaultCommand(ElevatorSubsystem.goToPoseCommand(elevatorSubsystem, armSubsystem, Constants.ElevatorConstants.defaultPosition));
    // armSubsystem.setDefaultCommand(new RunCommand(() -> armSubsystem.setGoal(armSubsystem.getRotation())));
    // intakeSubsystem.setDefaultCommand(intakeSubsystem.stopCommand());
    // routingSubsystem.setDefaultCommand(routingSubsystem.runCommand());
    // grabberSubsystem.setDefaultCommand(grabberSubsystem.intakeCommand());
    ledSubsystem.setDefaultCommand(ledSubsystem.setBlinkingCommand(Constants.LEDConstants.defaultColor, 0.5));
    // Configure the trigger bindings
    configureBindings();
    // Add testing buttons to dashboard
    addDashboardCommands();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    controller.rightStick().onTrue(new InstantCommand(() -> swerveSubsystem.zeroGyro()));
    // Reset modules to absolute on enable
    new Trigger(() -> DriverStation.isEnabled()).onTrue(
      new InstantCommand(() -> swerveSubsystem.resetModulesToAbsolute()).ignoringDisable(true));
    // Reset odometry to vision measurement when we first see a vision target
    new Trigger(() -> swerveSubsystem.hasTargets() && !swerveSubsystem.hasResetOdometry).onTrue(swerveSubsystem.resetIfTargets());

    new Trigger(() -> controller.getHID().getPOV() != -1).whileTrue(swerveSubsystem.headingLockDriveCommand(
      () -> -(Math.abs(Math.pow(controller.getLeftY(), 2)) + 0.05) * Math.signum(controller.getLeftY()), 
      () -> -(Math.abs(Math.pow(controller.getLeftX(), 2)) + 0.05) * Math.signum(controller.getLeftX()),  
      () -> (Math.PI * 2) - Math.toRadians(controller.getHID().getPOV()), 
      true, 
      true));


    // controller.leftBumper().whileTrue(intakeSubsystem.runCommand());
    
  }

  private void addDashboardCommands() {
    // SmartDashboard.putData("Path 1", ElevatorSubsystem.followLineCommand(
    //   elevatorSubsystem, 
    //   armSubsystem, 
    //   10, 
    //   10, 
    //   30, 
    //   15));

    // SmartDashboard.putData("Path 2", ElevatorSubsystem.followLineCommand(
    //   elevatorSubsystem, 
    //   armSubsystem, 
    //   30, 
    //   15, 
    //   10, 
    //   10));

    // SmartDashboard.putData("Sequence", ElevatorSubsystem.followLinearTrajectoryCommand(
    //   elevatorSubsystem, 
    //   armSubsystem, 
    //   List.of(
    //   Pair.of(new Translation2d(10, 10), 2.0),
    //   Pair.of(new Translation2d(20, 10), 2.0),
    //   Pair.of(new Translation2d(20, 15), 2.0),
    //   Pair.of(new Translation2d(30, 15), 2.0))));

    // var waypoints = new ArrayList<Translation2d>();
    // waypoints.add(new Translation2d(20, 10));
    // waypoints.add(new Translation2d(20, 20));
    // SmartDashboard.putData("Spline", ElevatorSubsystem.followSplineCommand(
    //   elevatorSubsystem, 
    //   armSubsystem, 
    //   SplineHelper.getCubicControlVectorsFromWaypoints(
    //     new Pose2d(10, 10, new Rotation2d()),
    //     waypoints.toArray(),
    //     new Pose2d(40, 30, new Rotation2d())
    //   )));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example path will be run in autonomous
    return new InstantCommand(() -> {});//swerveSubsystem.followPathCommand(PathPlanner.loadPath("Test Path", Constants.AutoConstants.autoConstraints));
  }

  /** Hopefully only need to use for LEDS */
  public void disabledPeriodic() {
    ledSubsystem.setSolid(Constants.LEDConstants.defaultColor);
  }

  // public Command runRouting(){
  //   return routingSubsystem.runCommand().andThen(grabberSubsystem.intakeCommand());

  // }
}
