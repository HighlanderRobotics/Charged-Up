// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SimMode;
import frc.robot.commands.ChoreoAutoChooser;
import frc.robot.commands.PathplannerAutoChooser;
import frc.robot.subsystems.Elevator.ElevatorIOFalcon;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Grabber.GrabberSubsystem;
import frc.robot.subsystems.Grabber.GrabberSubsystem.GamePiece;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.Routing.RoutingSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem.ExtensionState;
import frc.robot.subsystems.Swerve.GyroIOPigeon;
import frc.robot.subsystems.Swerve.GyroIOSim;
import frc.robot.subsystems.Swerve.SwerveModuleIOFalcon;
import frc.robot.subsystems.Swerve.SwerveModuleIOSim;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private SwerveSubsystem swerveSubsystem =
      new SwerveSubsystem(
          Robot.isReal() || Constants.SIM_MODE == SimMode.REPLAY
              ? new SwerveModuleIOFalcon[] {
                new SwerveModuleIOFalcon(0, Constants.Swerve.Mod0.constants),
                new SwerveModuleIOFalcon(1, Constants.Swerve.Mod1.constants),
                new SwerveModuleIOFalcon(2, Constants.Swerve.Mod2.constants),
                new SwerveModuleIOFalcon(3, Constants.Swerve.Mod3.constants)
              }
              : new SwerveModuleIOSim[] {
                new SwerveModuleIOSim(0, Constants.Swerve.Mod0.constants),
                new SwerveModuleIOSim(1, Constants.Swerve.Mod1.constants),
                new SwerveModuleIOSim(2, Constants.Swerve.Mod2.constants),
                new SwerveModuleIOSim(3, Constants.Swerve.Mod3.constants)
              },
          Robot.isReal() || Constants.SIM_MODE == SimMode.REPLAY
              ? new GyroIOPigeon()
              : new GyroIOSim());
  private ElevatorSubsystem elevatorSubsystem =
      new ElevatorSubsystem(Robot.isReal() ? new ElevatorIOFalcon() : new ElevatorIOSim());
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
  private PathplannerAutoChooser pathplannerAutoChooser =
      new PathplannerAutoChooser(
          swerveSubsystem,
          intakeSubsystem,
          elevatorSubsystem,
          routingSubsystem,
          grabberSubsystem,
          superstructureSubsystem);
  private ChoreoAutoChooser choreoAutoChooser =
      new ChoreoAutoChooser(
          swerveSubsystem,
          intakeSubsystem,
          elevatorSubsystem,
          routingSubsystem,
          grabberSubsystem,
          superstructureSubsystem);

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

  Field2d field = new Field2d();

  enum AutoSystem {
    choreo,
    pathplanner
  }

  LoggedDashboardChooser<AutoSystem> autoSystemType =
      new LoggedDashboardChooser<>("Auto System Type");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Set default commands here
    swerveSubsystem.setDefaultCommand(
        swerveSubsystem.driveCommand(
            () -> modifyJoystickAxis(controller.getLeftY(), controller.getLeftTriggerAxis()),
            () -> modifyJoystickAxis(controller.getLeftX(), controller.getLeftTriggerAxis()),
            () -> modifyJoystickAxis(controller.getRightX(), controller.getLeftTriggerAxis()),
            true,
            false,
            true));
    // this is a little sus, might have to change logic to use subsystems separately or combine
    // routing and intake subsystem
    elevatorSubsystem.setDefaultCommand(
        elevatorSubsystem
            .extendToInchesCommand(1.0)
            .andThen(
                elevatorSubsystem.zeroElevator(),
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

    autoSystemType.addDefaultOption("Choreo", AutoSystem.choreo);
    autoSystemType.addOption("Path Planner", AutoSystem.pathplanner);

    SmartDashboard.putData(
        "pose test", swerveSubsystem.poseLockDriveCommand(
            () -> 3.3, () -> 1, () -> 180, true, false));
    
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
        .onTrue(
            new InstantCommand(
                () ->
                    swerveSubsystem.zeroGyro(
                        DriverStation.getAlliance() == Alliance.Red ? 180 : 0)));
    // Reset modules to absolute on enable
    new Trigger(() -> DriverStation.isEnabled())
        .onTrue(
            new InstantCommand(() -> swerveSubsystem.resetModulesToAbsolute())
                .ignoringDisable(true));
    new Trigger(() -> DriverStation.isEnabled())
        .onTrue(new InstantCommand(() -> swerveSubsystem.lockOutSwerve = false));

    new Trigger(() -> controller.getHID().getPOV() != -1)
        .whileTrue(
            swerveSubsystem.headingLockDriveCommand(
                () -> modifyJoystickAxis(controller.getLeftY(), controller.getLeftTriggerAxis()),
                () -> modifyJoystickAxis(controller.getLeftX(), controller.getLeftTriggerAxis()),
                () -> (Math.PI * 2) - Math.toRadians(controller.getHID().getPOV()),
                true,
                true));

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

    // Heading snaps
    controller
        .rightTrigger()
        .whileTrue(
            swerveSubsystem.headingLockDriveCommand(
                // Use normal translation
                () -> modifyJoystickAxis(controller.getLeftY(), controller.getLeftTriggerAxis()),
                () -> modifyJoystickAxis(controller.getLeftX(), controller.getLeftTriggerAxis()),
                () -> {
                  // Find rotation based on 3 "zones" on controller
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

    // Scoring command
    controller
        .leftTrigger()
        .whileTrue(
            superstructureSubsystem
                .waitExtendToGoal(() -> swerveSubsystem.getLevel(), 0.0)
                .andThen(new RunCommand(() -> {}))
                .alongWith(
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

    superstructureSubsystem.storeTrigger.whileTrue(
        routingSubsystem.stopCommand().withInterruptBehavior(InterruptionBehavior.kCancelSelf));
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

  // Use this method to periodically log data to advantagekit
  public void loggingPeriodic() {
    Logger.getInstance()
        .recordOutput(
            "Controller Left Y Adjusted",
            modifyJoystickAxis(controller.getLeftY(), controller.getRightTriggerAxis()));
    Logger.getInstance()
        .recordOutput(
            "Controller Left X Adjusted",
            modifyJoystickAxis(controller.getLeftX(), controller.getRightTriggerAxis()));
    Logger.getInstance()
        .recordOutput(
            "Controller Right X Adjusted",
            modifyJoystickAxis(controller.getRightX(), controller.getRightTriggerAxis()));
    SmartDashboard.putData("choreo field", field);
    // SmartDashboard.putData("rezero", new InstantCommand(() ->
    // grabberSubsystem.resetEncoderToZero()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    switch (autoSystemType.get()) {
      case choreo:
        return choreoAutoChooser.getAutonomousCommand();
      case pathplanner:
        return pathplannerAutoChooser.getAutoCommand();
      default:
        return null;
    }
  }

  /** Hopefully only need to use for LEDS */
  public void disabledPeriodic() {
    if (DriverStation.getAlliance() == Alliance.Invalid) {
      ledSubsystem.setSolid(Constants.LEDConstants.defaultColor);
    } else if (DriverStation.getAlliance() == Alliance.Red) {
      ledSubsystem.runColorAlong(
          Color.kRed,
          autoSystemType.get() == AutoSystem.choreo
              ? Constants.LEDConstants.defaultColor
              : Color.kSeaGreen,
          12,
          2);
    } else {
      ledSubsystem.runColorAlong(
          Color.kBlue,
          autoSystemType.get() == AutoSystem.choreo
              ? Constants.LEDConstants.defaultColor
              : Color.kSeaGreen,
          12,
          2);
    }
  }
}
