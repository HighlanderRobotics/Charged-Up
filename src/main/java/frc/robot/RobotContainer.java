// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.logging.LoggingWrapper;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoChooser;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Grabber.GrabberSubsystem;
import frc.robot.subsystems.Grabber.GrabberSubsystem.GamePiece;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.Routing.RoutingSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem.ExtensionState;
import frc.robot.subsystems.SwerveSubsystem;

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
  private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private RoutingSubsystem routingSubsystem = new RoutingSubsystem();
  private LEDSubsystem ledSubsystem = new LEDSubsystem();
  private GrabberSubsystem grabberSubsystem = new GrabberSubsystem();

  private SuperstructureSubsystem superstructureSubsystem =
      new SuperstructureSubsystem(
          intakeSubsystem,
          elevatorSubsystem,
          routingSubsystem,
          swerveSubsystem,
          grabberSubsystem,
          ledSubsystem);
  private AutoChooser autoChooser =
      new AutoChooser(
          swerveSubsystem,
          intakeSubsystem,
          elevatorSubsystem,
          routingSubsystem,
          grabberSubsystem,
          superstructureSubsystem);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController controller =
      new CommandXboxController(OperatorConstants.driverControllerPort);
  private final CommandXboxController operator =
      new CommandXboxController(OperatorConstants.operatorControllerPort);

  Trigger isExtended =
      new Trigger(
          () ->
              elevatorSubsystem.getExtensionInches() > 4.5
                  || Constants.ElevatorConstants.PIDController.getGoal().position > 4.5
                  || grabberSubsystem.getIsExtended());

  boolean shouldUseChute = true;
  boolean isUsingChute = false;

  double lastHeadingSnapAngle = 0;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // LoggingWrapper.shared.add("autoBalance", swerveSubsystem.autoBalance());
    // Set default commands here
    swerveSubsystem.setDefaultCommand(
        swerveSubsystem.driveCommand(
            () -> modifyJoystickAxis(controller.getLeftY(), controller.getLeftTriggerAxis()),
            () -> modifyJoystickAxis(controller.getLeftX(), controller.getLeftTriggerAxis()),
            () -> modifyJoystickAxis(controller.getRightX(), controller.getLeftTriggerAxis()),
            true,
            true,
            true));
    // this is a little sus, might have to change logic to use subsystems separately or combine
    // routing and intake subsystem
    elevatorSubsystem.setDefaultCommand(
        elevatorSubsystem
            .extendToInchesCommand(1.0)
            .andThen(
                elevatorSubsystem.zeroElevator(),
                new PrintCommand("Working"),
                new StartEndCommand(
                    () -> elevatorSubsystem.disable(),
                    () -> elevatorSubsystem.enable(),
                    elevatorSubsystem)));
    grabberSubsystem.setDefaultCommand(
        grabberSubsystem
            .resetPivotCommand()
            .unless(() -> grabberSubsystem.gamePiece == GamePiece.Cone)
            .andThen(grabberSubsystem.runToRoutingStopCommand()));
    intakeSubsystem.setDefaultCommand(
        new WaitCommand(0.7)
            .andThen(
                new ConditionalCommand(
                        intakeSubsystem.extendCommand(),
                        intakeSubsystem.stopCommand(),
                        () -> isExtended.getAsBoolean())
                    .repeatedly()));
    routingSubsystem.setDefaultCommand(
        routingSubsystem.runCommand().withTimeout(0.7).andThen(routingSubsystem.stopCommand()));
    superstructureSubsystem.setDefaultCommand(
        new InstantCommand(() -> {}, superstructureSubsystem));
    ledSubsystem.setDefaultCommand(
        ledSubsystem.setBlinkingCommand(
            Constants.LEDConstants.defaultColor,
            () -> 1.0 / (swerveSubsystem.getLevel().level * 2)));
    // Configure the trigger bindings
    configureBindings();
    // Add testing buttons to dashboard
    // addDashboardCommands();
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
    controller
        .rightStick()
        .and(controller.leftStick())
        .onTrue(new InstantCommand(() -> swerveSubsystem.zeroGyro()));
    // Reset modules to absolute on enable
    new Trigger(() -> DriverStation.isEnabled())
        .onTrue(
            new InstantCommand(() -> swerveSubsystem.resetModulesToAbsolute())
                .ignoringDisable(true));
    new Trigger(() -> DriverStation.isEnabled())
        .onTrue(new InstantCommand(() -> swerveSubsystem.lockOutSwerve = false));

    new Trigger(() -> DriverStation.isTeleopEnabled())
        .and(() -> DriverStation.getAlliance() == Alliance.Red)
        .onTrue(
            new InstantCommand(
                () ->
                    swerveSubsystem.zeroGyro((swerveSubsystem.getYaw().getDegrees() + 180) % 360)));
    // Reset odometry to vision measurement when we first see a vision target
    // new Trigger(() -> swerveSubsystem.hasTargets() && !swerveSubsystem.hasResetOdometry)
    //   .onTrue(swerveSubsystem.resetIfTargets().alongWith(new PrintCommand("reset from vision")));

    new Trigger(() -> controller.getHID().getPOV() != -1)
        .whileTrue(
            swerveSubsystem.headingLockDriveCommand(
                () -> modifyJoystickAxis(controller.getLeftY(), controller.getLeftTriggerAxis()),
                () -> modifyJoystickAxis(controller.getLeftX(), controller.getLeftTriggerAxis()),
                () -> (Math.PI * 2) - Math.toRadians(controller.getHID().getPOV()),
                true,
                true));

    // new Trigger(() -> swerveSubsystem.hasTargets()).whileTrue(ledSubsystem.setSolidCommand(new
    // Color8Bit(Color.kNavy)));
    new Trigger(() -> grabberSubsystem.gamePiece == GamePiece.Cone)
        .whileTrue(
            ledSubsystem.setBlinkingCommand(
                Color.kYellow, Color.kGreen, () -> 1.0 / (swerveSubsystem.getLevel().level * 2)));
    new Trigger(
            () -> grabberSubsystem.gamePiece == GamePiece.Cube && grabberSubsystem.getBeambreak())
        .whileTrue(
            ledSubsystem.setBlinkingCommand(
                Constants.LEDConstants.defaultColor,
                Color.kGreen,
                () -> 1.0 / (swerveSubsystem.getLevel().level * 2)));

    controller
        .leftBumper()
        .and(() -> !shouldUseChute)
        .whileTrue(
            superstructureSubsystem
                .waitExtendToInches(Constants.humanPlayerLevel)
                .andThen(new RunCommand(() -> {}, elevatorSubsystem)))
        .onTrue(
            grabberSubsystem
                .intakeConeDoubleCommand()
                .raceWith(
                    ledSubsystem.setBlinkingCommand(
                        Color.kYellow, () -> 1.0 / (swerveSubsystem.getLevel().level * 2))));

    controller
        .leftBumper()
        .and(() -> shouldUseChute)
        .whileTrue(
            grabberSubsystem
                .intakeConeSingleContinuousCommand()
                .repeatedly()
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .alongWith(
                    ledSubsystem
                        .setBlinkingCommand(
                            Color.kYellow, () -> 1.0 / (swerveSubsystem.getLevel().level * 2))
                        .until(() -> grabberSubsystem.gamePiece == GamePiece.Cone),
                    superstructureSubsystem
                        .waitExtendToInches(Constants.ScoringLevels.chuteLevel)
                        .andThen(
                            new InstantCommand(() -> isUsingChute = true),
                            new RunCommand(() -> {}, elevatorSubsystem)
                                .alongWith(intakeSubsystem.stopCommand()))))
        .onFalse(
            new InstantCommand(() -> isUsingChute = false)
                .andThen(new RunCommand(() -> {}, elevatorSubsystem).withTimeout(1.0))
                .raceWith(grabberSubsystem.intakeConeSingleCommand()));

    controller
        .rightBumper()
        .whileTrue(
            run(
                intakeSubsystem.runCommand(),
                routingSubsystem.runCommand(),
                grabberSubsystem.intakeCubeCommand()));
    operator
        .y()
        .whileTrue(
            new InstantCommand(
                () ->
                    swerveSubsystem.setLevel(
                        ElevatorSubsystem.ScoringLevels.L3,
                        grabberSubsystem.gamePiece == GamePiece.Cone)));
    operator
        .b()
        .whileTrue(
            new InstantCommand(
                () ->
                    swerveSubsystem.setLevel(
                        ElevatorSubsystem.ScoringLevels.L2,
                        grabberSubsystem.gamePiece == GamePiece.Cone)));
    operator
        .a()
        .whileTrue(
            new InstantCommand(
                () ->
                    swerveSubsystem.setLevel(
                        ElevatorSubsystem.ScoringLevels.L1,
                        grabberSubsystem.gamePiece == GamePiece.Cone)));
    operator.x().whileTrue(swerveSubsystem.autoBalance());

    operator
        .povDown()
        .onTrue(new InstantCommand(() -> grabberSubsystem.gamePiece = GamePiece.Cone));
    operator
        .povCenter()
        .onTrue(new InstantCommand(() -> grabberSubsystem.gamePiece = GamePiece.None));
    operator.povUp().onTrue(new InstantCommand(() -> grabberSubsystem.gamePiece = GamePiece.Cube));
    operator.leftBumper().onTrue(new InstantCommand(() -> shouldUseChute = false));
    operator.rightBumper().onTrue(new InstantCommand(() -> shouldUseChute = true));
    operator.start().whileTrue(grabberSubsystem.resetPivotCommand());
    // controller.a().whileTrue(new ScoringCommand(ScoringLevels.L1, () -> 0, elevatorSubsystem,
    // swerveSubsystem, grabberSubsystem,
    // superstructureSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    // controller.b().whileTrue(new ScoringCommand(ScoringLevels.L2, () -> 0, elevatorSubsystem,
    // swerveSubsystem, grabberSubsystem,
    // superstructureSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    // controller.y().whileTrue(new ScoringCommand(ScoringLevels.L3, () -> 0, elevatorSubsystem,
    // swerveSubsystem, grabberSubsystem,
    // superstructureSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    controller
        .rightTrigger()
        .whileTrue(
            swerveSubsystem.headingLockDriveCommand(
                () -> modifyJoystickAxis(controller.getLeftY(), controller.getLeftTriggerAxis()),
                () -> modifyJoystickAxis(controller.getLeftX(), controller.getLeftTriggerAxis()),
                () -> {
                  var joystickRotation =
                      new Rotation2d(controller.getRightX(), -controller.getRightY());
                  SmartDashboard.putNumber("heading snap", lastHeadingSnapAngle);
                  SmartDashboard.putNumber("heading joystick rot", joystickRotation.getRotations());
                  if (Math.sqrt(
                          (controller.getRightX() * controller.getRightX())
                              + (controller.getRightY() * controller.getRightY()))
                      < 0.2) {
                    return lastHeadingSnapAngle;
                  }

                  double correctedJoystickRot =
                      joystickRotation.getRotations() > 0
                          ? joystickRotation.getRotations()
                          : 1.0 + joystickRotation.getRotations();
                  SmartDashboard.putNumber("heading corrected joystick rot", correctedJoystickRot);

                  if (correctedJoystickRot < 0.917 && correctedJoystickRot > 0.583) {
                    SmartDashboard.putString("heading snap id", "score");
                    lastHeadingSnapAngle = Math.PI;
                  } else if (correctedJoystickRot < 0.583 && correctedJoystickRot > 0.25) {
                    SmartDashboard.putString("heading snap id", "left");
                    lastHeadingSnapAngle = Math.PI / 2;
                  } else if (correctedJoystickRot < 0.25 || correctedJoystickRot > 0.917) {
                    SmartDashboard.putString("heading snap id", "right");
                    lastHeadingSnapAngle = Math.PI + (Math.PI / 2);
                  }

                  return lastHeadingSnapAngle;
                },
                true,
                true));

    controller
        .leftTrigger()
        .whileTrue(
            swerveSubsystem.driveCommand(
                () -> modifyJoystickAxis(controller.getLeftY(), controller.getLeftTriggerAxis()),
                () -> modifyJoystickAxis(controller.getLeftX(), controller.getLeftTriggerAxis()),
                () -> modifyJoystickAxis(controller.getRightX(), controller.getLeftTriggerAxis()),
                true,
                false,
                true));
    controller
        .leftTrigger()
        .whileTrue(
            superstructureSubsystem
                .waitExtendToGoal(() -> swerveSubsystem.getLevel(), 0.0)
                .andThen(new RunCommand(() -> {}))
                .alongWith( // ledSubsystem.setRainbowCommand(),
                    routingSubsystem.slowRunCommand(),
                    new ConditionalCommand(
                            new RunCommand(() -> grabberSubsystem.holdCube(), grabberSubsystem)
                                .withTimeout(0.5),
                            new RunCommand(() -> grabberSubsystem.intakeCone(), grabberSubsystem)
                                .withTimeout(0.2),
                            () -> grabberSubsystem.gamePiece == GamePiece.Cube)
                        .withTimeout(0.5)
                        .andThen(grabberSubsystem.runToScoringCommand())))
        .onFalse(
            new ConditionalCommand(
                    grabberSubsystem.scoreCubeCommand(),
                    grabberSubsystem.scoreConeCommand(),
                    () -> grabberSubsystem.gamePiece == GamePiece.Cube)
                .raceWith(new RunCommand(() -> {}, elevatorSubsystem))
                .unless(() -> elevatorSubsystem.getExtensionInches() < 10.0));

    controller
        .x()
        .whileTrue(
            (run(
                intakeSubsystem.outakeCommand(),
                routingSubsystem.outakeCommand(),
                grabberSubsystem.outakeCubeCommand())));
    controller
        .y()
        .whileTrue(
            swerveSubsystem
                .autoBalanceVelocity()
                .alongWith(ledSubsystem.setBlinkingCommand(Color.kBlue, 0.5)));

    controller
        .start()
        .whileTrue(
            elevatorSubsystem
                .extendToInchesCommand(-2)
                .until(() -> elevatorSubsystem.getLimitSwitch())
                .andThen(new PrintCommand("reset elevator")));
    controller.back().whileTrue(run(grabberSubsystem.intakeConeSingleCommand()));

    operator.back().whileTrue(run(grabberSubsystem.intakeConeSingleCommand()));

    operator
        .leftTrigger()
        .and(() -> operator.rightTrigger().getAsBoolean())
        .whileTrue(
            superstructureSubsystem
                .waitExtendToInches(Constants.ScoringLevels.chuteLevel)
                .andThen(
                    new InstantCommand(() -> isUsingChute = true),
                    new RunCommand(() -> {}, elevatorSubsystem)
                        .alongWith(intakeSubsystem.outakeCommand()))
                .alongWith(routingSubsystem.outakeCommand()));

    isExtended.whileFalse(
        new ConditionalCommand(
            new RunCommand(() -> superstructureSubsystem.setMode(ExtensionState.STORE)),
            new RunCommand(() -> superstructureSubsystem.setMode(ExtensionState.RETRACT_AND_ROUTE)),
            () -> grabberSubsystem.gamePiece == GamePiece.Cone));

    // isExtended.whileTrue(
    //   intakeSubsystem.extendCommand().unless(() ->
    // isUsingChute).repeatedly()//.withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
    //   );

    superstructureSubsystem.retractAndRouteTrigger.whileTrue(
        run(
            // elevatorSubsystem.extendToInchesCommand(0.0),
            // grabberSubsystem.runToStorageCommand().withInterruptBehavior(InterruptionBehavior.kCancelSelf)
            ));

    superstructureSubsystem.storeTrigger.whileTrue(
        run(routingSubsystem.stopCommand()
            // armSubsystem.runToHorizontalCommand()
            // elevatorSubsystem.extendToInchesCommand(0.0)
            )
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
  }

  // Uses old logging system
  @Deprecated
  private void addDashboardCommands() {

    LoggingWrapper.shared.add("intake toggle", intakeSubsystem.extendCommand());

    LoggingWrapper.shared.add("extend to 0", superstructureSubsystem.waitExtendToInches(0));
    LoggingWrapper.shared.add("extend to 12", superstructureSubsystem.waitExtendToInches(12));
    LoggingWrapper.shared.add("extend to 36", superstructureSubsystem.waitExtendToInches(36));

    LoggingWrapper.shared.add(
        "reset elevator",
        new InstantCommand(() -> elevatorSubsystem.zeroMotor(), elevatorSubsystem)
            .ignoringDisable(true));

    LoggingWrapper.shared.add(
        "reset to 0",
        new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d()), swerveSubsystem));

    LoggingWrapper.shared.add(
        "rezero elevator", new InstantCommand(() -> elevatorSubsystem.zeroMotor()));

    LoggingWrapper.shared.add(
        "Rezero Grabber",
        grabberSubsystem.resetPivotCommand().alongWith(intakeSubsystem.extendCommand()));

    LoggingWrapper.shared.add(
        "Run grabber to storage",
        grabberSubsystem
            .runToStorageCommand()
            .alongWith(intakeSubsystem.extendCommand().repeatedly()));
    LoggingWrapper.shared.add(
        "Run grabber to routing",
        grabberSubsystem
            .runToRoutingCommand()
            .alongWith(intakeSubsystem.extendCommand().repeatedly()));
    LoggingWrapper.shared.add(
        "Run grabber to scoring",
        grabberSubsystem
            .runToScoringCommand()
            .alongWith(intakeSubsystem.extendCommand().repeatedly()));
    LoggingWrapper.shared.add(
        "Run grabber to double substation",
        grabberSubsystem
            .runToDoubleSubstationCommand()
            .alongWith(intakeSubsystem.extendCommand().repeatedly()));
    LoggingWrapper.shared.add(
        "Run grabber to single substation",
        grabberSubsystem
            .runToSingleSubstationCommand()
            .alongWith(intakeSubsystem.extendCommand().repeatedly()));

    LoggingWrapper.shared.add(
        "Grabber intake cone single substation no extend",
        grabberSubsystem.intakeConeSingleCommand().alongWith(intakeSubsystem.extendCommand()));
    LoggingWrapper.shared.add(
        "Grabber intake cone double substation no extend",
        grabberSubsystem.intakeConeDoubleCommand().alongWith(intakeSubsystem.extendCommand()));
    LoggingWrapper.shared.add(
        "Grabber intake cube no extend", grabberSubsystem.intakeCubeCommand());

    LoggingWrapper.shared.add(
        "Grabber outake cone",
        grabberSubsystem
            .outakeConeCommand()
            .alongWith(intakeSubsystem.extendCommand())
            .withTimeout(1.0));
  }

  private static Command run(Command... commands) {
    return new ParallelCommandGroup(commands);
  }

  private double modifyJoystickAxis(double joystick, double fineTuneAxis) {
    return -(Math.abs(Math.pow(joystick, 2)) + 0.05)
        * Math.signum(joystick)
        * (1 - (0.5 * fineTuneAxis))
        * (1 - (0.5 * (controller.leftBumper().getAsBoolean() ? 0.4 : 0)));
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
    // ledSubsystem.setNoise(Constants.LEDConstants.defaultColor, new Color8Bit(Color.kBlack), (int)
    // (Timer.getFPGATimestamp() * 20));
    if (DriverStation.getAlliance() == Alliance.Invalid) {
      ledSubsystem.setSolid(Constants.LEDConstants.defaultColor);
    } else if (DriverStation.getAlliance() == Alliance.Red) {
      ledSubsystem.runColorAlong(Color.kRed, Constants.LEDConstants.defaultColor, 12, 2);
    } else {
      ledSubsystem.runColorAlong(Color.kBlue, Constants.LEDConstants.defaultColor, 12, 2);
    }
  }
}
