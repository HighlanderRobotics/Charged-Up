// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoChooser;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.commands.ScoringCommand;
import frc.robot.subsystems.RoutingSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import frc.robot.subsystems.ElevatorSubsystem.ScoringLevels;
import frc.robot.subsystems.SuperstructureSubsystem.ExtensionState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private LEDSubsystem ledSubsystem = new LEDSubsystem();
  
  private SuperstructureSubsystem superstructureSubsystem = 
  new SuperstructureSubsystem(intakeSubsystem, elevatorSubsystem, armSubsystem, routingSubsystem, grabberSubsystem, swerveSubsystem, ledSubsystem);
  // private LEDSubsystem ledSubsystem = new LEDSubsystem();
  private AutoChooser autoChooser = new AutoChooser(swerveSubsystem, intakeSubsystem, elevatorSubsystem, armSubsystem, grabberSubsystem, routingSubsystem, superstructureSubsystem);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController controller =
      new CommandXboxController(OperatorConstants.driverControllerPort);
  private final CommandXboxController operator =
      new CommandXboxController(OperatorConstants.operatorControllerPort);

  Trigger isExtended = new Trigger(() -> elevatorSubsystem.getExtensionInches() > 4.5 || Constants.ElevatorConstants.PIDController.getGoal().position > 4.5);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
      SmartDashboard.putData("autoBalance", swerveSubsystem.autoBalance());
    // Set default commands here
    swerveSubsystem.setDefaultCommand(swerveSubsystem.driveCommand(
      () -> modifyJoystickAxis(controller.getLeftY(), controller.getLeftTriggerAxis()), 
      () -> modifyJoystickAxis(controller.getLeftX(), controller.getLeftTriggerAxis()), 
      () -> modifyJoystickAxis(controller.getRightX(), controller.getLeftTriggerAxis()), 
      true, 
      true,
      true));
    // this is a little sus, might have to change logic to use subsystems separately or combine routing and intake subsystem
    elevatorSubsystem.setDefaultCommand(
      elevatorSubsystem.extendToInchesCommand(1.0)
      .andThen(elevatorSubsystem.zeroElevator().repeatedly()
        ));
    armSubsystem.setDefaultCommand(new RunCommand(() -> armSubsystem.stop(), armSubsystem));
    intakeSubsystem.setDefaultCommand(new WaitCommand(0.7)
      .andThen(new ConditionalCommand(
        intakeSubsystem.extendCommand(), 
        intakeSubsystem.stopCommand(), 
        () -> isExtended.getAsBoolean()
      ).repeatedly()));
    routingSubsystem.setDefaultCommand(routingSubsystem.runCommand().withTimeout(0.7).andThen(routingSubsystem.stopCommand()));
    grabberSubsystem.setDefaultCommand(grabberSubsystem.stopCommand());
    superstructureSubsystem.setDefaultCommand(new InstantCommand(() -> {}, superstructureSubsystem));
    ledSubsystem.setDefaultCommand(ledSubsystem.setBlinkingCommand(Constants.LEDConstants.defaultColor, () -> 1.0 / (swerveSubsystem.getLevel().level * 2)));
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
    new Trigger(() -> DriverStation.isEnabled()).onTrue(new InstantCommand(() -> swerveSubsystem.lockOutSwerve = false));
    // Reset odometry to vision measurement when we first see a vision target
    // new Trigger(() -> swerveSubsystem.hasTargets() && !swerveSubsystem.hasResetOdometry)
    //   .onTrue(swerveSubsystem.resetIfTargets().alongWith(new PrintCommand("reset from vision")));

    new Trigger(() -> controller.getHID().getPOV() != -1).whileTrue(swerveSubsystem.headingLockDriveCommand(
      () -> modifyJoystickAxis(controller.getLeftY(), controller.getLeftTriggerAxis()), 
      () -> modifyJoystickAxis(controller.getLeftX(), controller.getLeftTriggerAxis()),
      () -> (Math.PI * 2) - Math.toRadians(controller.getHID().getPOV()), 
      true, 
      true));
    
    // new Trigger(() -> swerveSubsystem.hasTargets()).whileTrue(ledSubsystem.setSolidCommand(new Color8Bit(Color.kNavy)));
    new Trigger(() -> swerveSubsystem.checkIfConeGoalWithOverride())
      .whileTrue(
        ledSubsystem.setBlinkingCommand(new Color8Bit(Color.kYellow), () -> 1.0 / (swerveSubsystem.getLevel().level * 2)));
    controller.leftBumper().whileTrue(
      superstructureSubsystem.waitExtendToInches(Constants.humanPlayerLevel)
      .andThen(new RunCommand(() -> {}, elevatorSubsystem)
      .alongWith(grabberSubsystem.intakeClosedCommand(), swerveSubsystem.setGamePieceOverride(true))));
    controller.rightBumper().whileTrue(run(
      intakeSubsystem.runCommand(), 
      routingSubsystem.runCommand(), 
      grabberSubsystem.intakeOpenCommand(),
      armSubsystem.runToRoutingCommand(),
      swerveSubsystem.setGamePieceOverride(false)));
    operator.y().whileTrue(new InstantCommand (() -> swerveSubsystem.setLevel(ElevatorSubsystem.ScoringLevels.L3)));
    operator.b().whileTrue(new InstantCommand (() -> swerveSubsystem.setLevel(ElevatorSubsystem.ScoringLevels.L2)));
    operator.a().whileTrue(new InstantCommand (() -> swerveSubsystem.setLevel(ElevatorSubsystem.ScoringLevels.L1)));
    operator.x().whileTrue(swerveSubsystem.autoBalance());
    // operator.leftBumper().onTrue(swerveSubsystem.setGamePieceOverride(true));
    // operator.rightBumper().onTrue(swerveSubsystem.setGamePieceOverride(false));
    // controller.a().whileTrue(new ScoringCommand(ScoringLevels.L1, () -> 0, elevatorSubsystem, swerveSubsystem, grabberSubsystem, superstructureSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    // controller.b().whileTrue(new ScoringCommand(ScoringLevels.L2, () -> 0, elevatorSubsystem, swerveSubsystem, grabberSubsystem, superstructureSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    // controller.y().whileTrue(new ScoringCommand(ScoringLevels.L3, () -> 0, elevatorSubsystem, swerveSubsystem, grabberSubsystem, superstructureSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    controller.leftTrigger().whileTrue(
      superstructureSubsystem.waitExtendToGoal(() -> swerveSubsystem.getLevel()).andThen(new RunCommand(() -> {}))
      .alongWith(ledSubsystem.setRainbowCommand(), grabberSubsystem.closeCommand()))
      .onFalse(
        new ConditionalCommand(
          grabberSubsystem.susL3Command()
          .raceWith(new RunCommand(() -> {}, elevatorSubsystem))
          .withTimeout(0.5)
          .andThen(new WaitCommand(0.2), grabberSubsystem.openCommand())
          .andThen(new WaitCommand(0.2)), 
          new ConditionalCommand(
            grabberSubsystem.openCommand().andThen(new WaitCommand(0.2)), 
            grabberSubsystem.outakeOpenCommand().withTimeout(1.0), 
            () -> swerveSubsystem.isConeOveride), 
          () -> swerveSubsystem.isConeOveride && swerveSubsystem.getLevel() == ScoringLevels.L3
          )
          .unless(() -> elevatorSubsystem.getExtensionInches() < 10.0)
          // .andThen(swerveSubsystem.disableGamePieceOverride())
    );
    controller.rightTrigger().whileTrue(swerveSubsystem.tapeDriveAssistCommand(
      () -> modifyJoystickAxis(controller.getLeftY(), controller.getLeftTriggerAxis()))
      .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    controller.x().whileTrue((run(intakeSubsystem.outakeCommand(), routingSubsystem.outakeCommand(), grabberSubsystem.outakeCommand())));
    
    controller.start().whileTrue(elevatorSubsystem.extendToInchesCommand(-2)
      .until(() -> elevatorSubsystem.limitSwitch.get())
      .andThen(new PrintCommand("reset elevator")));
    controller.back().onTrue(grabberSubsystem.closeCommand());

    isExtended.whileFalse(new ConditionalCommand(
        new RunCommand(() -> superstructureSubsystem.setMode(ExtensionState.STORE)), 
        new RunCommand(() -> superstructureSubsystem.setMode(ExtensionState.RETRACT_AND_ROUTE)), 
        () -> grabberSubsystem.hasGamePiece()));

    isExtended.whileTrue(run(
      intakeSubsystem.extendCommand().repeatedly().withInterruptBehavior(InterruptionBehavior.kCancelIncoming),
      armSubsystem.runToHorizontalCommand()));

    superstructureSubsystem.retractAndRouteTrigger.whileTrue(run(
      //elevatorSubsystem.extendToInchesCommand(0.0),
      armSubsystem.runToRoutingCommand()));

    superstructureSubsystem.storeTrigger.whileTrue(run(
      routingSubsystem.stopCommand()
      // armSubsystem.runToHorizontalCommand()
      //elevatorSubsystem.extendToInchesCommand(0.0)
    ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
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
    
    SmartDashboard.putData("reset to vision", swerveSubsystem.resetIfTargets());
    SmartDashboard.putData("reset to 0", new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d()), swerveSubsystem));
  
    SmartDashboard.putData("arm to -0.5", armSubsystem.runToRotationCommand(-0.5));
    SmartDashboard.putData("arm to -1.3", armSubsystem.runToRotationCommand(-1.3));
    SmartDashboard.putData("arm to horizontal", armSubsystem.runToHorizontalCommand());
    SmartDashboard.putData("arm to routing", armSubsystem.runToRoutingCommand());

    SmartDashboard.putData("rezero elevator", new InstantCommand(() -> elevatorSubsystem.zeroMotor()));

    SmartDashboard.putData("scoring sequence", new ScoringCommand(ScoringLevels.L3, () -> 0, elevatorSubsystem, swerveSubsystem, grabberSubsystem, superstructureSubsystem));
    SmartDashboard.putData("driver assist tape allign", 
      swerveSubsystem.tapeDriveAssistCommand(
        () -> modifyJoystickAxis(controller.getLeftY(), controller.getLeftTriggerAxis())));
  }

  private static Command run(Command ... commands) {
    return new ParallelCommandGroup(commands);
  }

  private double modifyJoystickAxis(double joystick, double fineTuneAxis) {
    return -(Math.abs(Math.pow(joystick, 2)) + 0.05) * Math.signum(joystick) * (1 - (0.5 * fineTuneAxis));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example path will be run in autonomous
      return autoChooser.getAutoCommand();
  }


    
      
    

  /** Hopefully only need to use for LEDS */
  public void disabledPeriodic() {
    // ledSubsystem.setNoise(Constants.LEDConstants.defaultColor, new Color8Bit(Color.kBlack), (int) (Timer.getFPGATimestamp() * 20));
    ledSubsystem.setSolid(Constants.LEDConstants.defaultColor);
  }
}