// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.EatingCommand;
import frc.robot.commands.ScoringCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RoutingSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Level;

import java.util.List;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private ArmSubsystem armSubsystem = new ArmSubsystem();
  private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private RoutingSubsystem routingSubsystem = new RoutingSubsystem();
  private GrabberSubsystem grabberSubsystem = new GrabberSubsystem();
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
    // Configure the trigger bindings
    configureBindings();
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

    controller.a().whileTrue(new InstantCommand(() -> elevatorSubsystem.setTargetLevel(Level.L3)));
    controller.b().whileTrue(new InstantCommand(() -> elevatorSubsystem.setTargetLevel(Level.L2)));
    controller.y().whileTrue(new InstantCommand(() -> elevatorSubsystem.setTargetLevel(Level.L1)));
    controller.x().whileTrue(new EatingCommand(elevatorSubsystem, armSubsystem, swerveSubsystem, grabberSubsystem));
    
    controller.rightBumper().whileTrue(
      new ScoringCommand(elevatorSubsystem.getTargetLevel(), elevatorSubsystem, armSubsystem, swerveSubsystem, grabberSubsystem));
    

    controller.leftBumper().whileTrue(intakeSubsystem.runCommand());

    
  }

  private void addDashboardCommands() {
    SmartDashboard.putData("Path 1", ElevatorSubsystem.followLineCommand(
      elevatorSubsystem, 
      armSubsystem, 
      10, 
      10, 
      30, 
      15));

    SmartDashboard.putData("Path 2", ElevatorSubsystem.followLineCommand(
      elevatorSubsystem, 
      armSubsystem, 
      30, 
      15, 
      10, 
      10));

    SmartDashboard.putData("Sequence", ElevatorSubsystem.followLinearTrajectoryCommand(
      elevatorSubsystem, 
      armSubsystem, 
      List.of(
      Pair.of(new Translation2d(10, 10), 2.0),
      Pair.of(new Translation2d(20, 10), 2.0),
      Pair.of(new Translation2d(20, 15), 2.0),
      Pair.of(new Translation2d(30, 15), 2.0))));
    SmartDashboard.putData("Scoring Sequence", new ScoringCommand(Level.L3, elevatorSubsystem, armSubsystem, swerveSubsystem, grabberSubsystem));
    SmartDashboard.putData("Odometry Reset",  new InstantCommand (() -> swerveSubsystem.resetOdometry(new Pose2d())));
    SmartDashboard.putData("testpath reset odometry", new InstantCommand (() -> swerveSubsystem.resetOdometry(PathPlanner.loadPath("Test Path", Constants.AutoConstants.autoConstraints).getInitialHolonomicPose()), swerveSubsystem));
    
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
  } 
}
