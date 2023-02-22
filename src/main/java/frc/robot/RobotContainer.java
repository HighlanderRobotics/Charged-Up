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
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem.ExtensionState;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.ElevatorSubsystem.Level;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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

  private SuperstructureSubsystem superstructureSubsystem = 
    new SuperstructureSubsystem(intakeSubsystem, elevatorSubsystem, armSubsystem, routingSubsystem, grabberSubsystem);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController controller =
      new CommandXboxController(OperatorConstants.driverControllerPort);

  Trigger isExtended = new Trigger(() -> elevatorSubsystem.getExtensionInches() > 6 || Constants.ElevatorConstants.PIDController.getGoal().position > 6);

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
    elevatorSubsystem.setDefaultCommand(elevatorSubsystem.extendToInchesCommand(0.5));
    armSubsystem.setDefaultCommand(new RunCommand(() -> armSubsystem.stop(), armSubsystem));
    intakeSubsystem.setDefaultCommand(new ConditionalCommand(intakeSubsystem.extendCommand(), intakeSubsystem.stopCommand(), () -> isExtended.getAsBoolean()).repeatedly());
    routingSubsystem.setDefaultCommand(routingSubsystem.stopCommand());
    grabberSubsystem.setDefaultCommand(grabberSubsystem.stopCommand());
    superstructureSubsystem.setDefaultCommand(new InstantCommand(() -> {}, superstructureSubsystem));
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

    controller.leftBumper().whileTrue(superstructureSubsystem.waitExtendToInches(36));
    controller.rightBumper().whileTrue(run(intakeSubsystem.runCommand(), routingSubsystem.runCommand(), grabberSubsystem.intakeCommand()));
    controller.b().whileTrue(new ScoringCommand(Level.L3, elevatorSubsystem, armSubsystem, swerveSubsystem, grabberSubsystem));
    controller.a().whileTrue(run(intakeSubsystem.outakeCommand(), routingSubsystem.outakeCommand(), grabberSubsystem.outakeCommand()));

    isExtended
      .whileTrue(new RunCommand(() -> superstructureSubsystem.setMode(ExtensionState.EXTEND)))
      .whileFalse(new ConditionalCommand(
        new RunCommand(() -> superstructureSubsystem.setMode(ExtensionState.STORE)), 
        new RunCommand(() -> superstructureSubsystem.setMode(ExtensionState.RETRACT_AND_ROUTE)), 
        () -> grabberSubsystem.hasGamePiece()));

    isExtended.whileTrue(intakeSubsystem.extendCommand().repeatedly().withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    superstructureSubsystem.retractAndRouteTrigger.whileTrue(run(
      //elevatorSubsystem.extendToInchesCommand(0.0),
      armSubsystem.runToRotationCommand(new Rotation2d(2)))); // TODO: Find rotation

    superstructureSubsystem.storeTrigger.whileTrue(run(
      routingSubsystem.stopCommand(),
      grabberSubsystem.intakeCommand()
      //elevatorSubsystem.extendToInchesCommand(0.0)
    ));
  }

  private void addDashboardCommands() {
    SmartDashboard.putBoolean("Grabber has thing", grabberSubsystem.hasGamePiece());

    SmartDashboard.putData("intake toggle", intakeSubsystem.extendCommand());

    SmartDashboard.putData("extend to 0", superstructureSubsystem.waitExtendToInches(0));
    SmartDashboard.putData("extend to 12", superstructureSubsystem.waitExtendToInches(12));
    SmartDashboard.putData("extend to 36", superstructureSubsystem.waitExtendToInches(36));

    SmartDashboard.putData("reset elevator", new InstantCommand(() -> elevatorSubsystem.zeroMotor(), elevatorSubsystem).ignoringDisable(true));
    
    SmartDashboard.putData("jog arm up", new RunCommand(() -> armSubsystem.jogUp(), armSubsystem));
    SmartDashboard.putData("jog arm down", new RunCommand(() -> armSubsystem.jogDown(), armSubsystem));
  }

  private static Command run(Command ... commands) {
    return new ParallelCommandGroup(commands);
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
