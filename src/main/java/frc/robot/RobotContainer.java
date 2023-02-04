// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ElevatorCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    SmartDashboard.putData("Scoring Sequence", new ElevatorCommand(elevatorSubsystem, swerveSubsystem));
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

    controller.y().onTrue(new InstantCommand(() -> elevatorSubsystem.pickTopLevel()));
    controller.b().onTrue(new InstantCommand(() -> elevatorSubsystem.pickMidLevel()));
    controller.a().onTrue(new InstantCommand(() -> elevatorSubsystem.pickBottomLevel()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example path will be run in autonomous
    return swerveSubsystem.followPathCommand(PathPlanner.loadPath("Test Path", Constants.AutoConstants.autoConstraints));
  }

  /** Hopefully only need to use for LEDS */
  public void disabledPeriodic() {
  }


  
  
}
