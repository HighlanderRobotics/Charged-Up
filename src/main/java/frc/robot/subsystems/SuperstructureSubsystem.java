// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem.ScoringLevels;
import frc.robot.subsystems.Grabber.GrabberSubsystem;
import frc.robot.subsystems.Grabber.GrabberSubsystem.GamePiece;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.Routing.RoutingSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SuperstructureSubsystem extends SubsystemBase {
  IntakeSubsystem intakeSubsystem;
  ElevatorSubsystem elevatorSubsystem;
  RoutingSubsystem routingSubsystem;
  SwerveSubsystem swerveSubsystem;
  GrabberSubsystem grabberSubsystem;
  LEDSubsystem ledSubsystem;

  ExtensionState mode = ExtensionState.RETRACT_AND_ROUTE;

  public Trigger retractAndRouteTrigger =
      new Trigger(() -> mode == ExtensionState.RETRACT_AND_ROUTE);
  public Trigger storeTrigger = new Trigger(() -> mode == ExtensionState.STORE);
  public Trigger extendTrigger = new Trigger(() -> mode == ExtensionState.EXTEND);

  /** Creates a new SuperstructureSubsystem. */
  public SuperstructureSubsystem(
      IntakeSubsystem intakeSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      RoutingSubsystem routingSubsystem,
      SwerveSubsystem swerveSubsystem,
      GrabberSubsystem grabberSubsystem,
      LEDSubsystem ledSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.routingSubsystem = routingSubsystem;
    this.grabberSubsystem = grabberSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.ledSubsystem = ledSubsystem;
  }

  public void setMode(ExtensionState mode) {
    this.mode = mode;
  }

  public CommandBase waitExtendToInches(DoubleSupplier extensionInches) {
    return intakeSubsystem
        .extendCommand()
        .andThen(new WaitCommand(0.3))
        .andThen(elevatorSubsystem.extendToInchesCommand(extensionInches))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public CommandBase waitExtendToInches(double extensionInches) {
    return waitExtendToInches(() -> extensionInches);
  }

  public CommandBase waitExtendToGoal(Supplier<ScoringLevels> level, double adjust) {
    return new PrintCommand(
            "extension target "
                + swerveSubsystem.getExtension(
                    level.get(), grabberSubsystem.gamePiece == GamePiece.Cone)
                + adjust)
        .andThen(
            waitExtendToInches(
                () ->
                    swerveSubsystem.getExtension(
                            level.get(), grabberSubsystem.gamePiece == GamePiece.Cone)
                        + adjust));
  }

  public CommandBase waitExtendToGoal(ScoringLevels level, double adjust) {
    return waitExtendToGoal(() -> level, adjust);
  }

  public CommandBase waitExtendToGoal(double adjust) {
    return waitExtendToGoal(swerveSubsystem.getLevel(), adjust);
  }

  public CommandBase waitExtendToGoal() {
    return waitExtendToGoal(swerveSubsystem.getLevel(), 0);
  }

  public CommandBase scoreNoAim() {
    return routingSubsystem
        .stopCommand()
        .raceWith(
            this.waitExtendToGoal(() -> swerveSubsystem.getLevel(), -1.0)
                .deadlineWith(
                    ledSubsystem.setRainbowCommand(),
                    new WaitCommand(0.1)
                        .andThen(
                            grabberSubsystem.stopCommand().withTimeout(0.1),
                            new ConditionalCommand(
                              grabberSubsystem.runToScoringHoldConeCommand(), 
                              grabberSubsystem.stopCommand(),
                              () -> grabberSubsystem.gamePiece == GamePiece.Cone)))
                .withTimeout(1.2)
                .andThen(
                    new WaitCommand(.15),
                    new ConditionalCommand(
                            grabberSubsystem.scoreCubeCommand(),
                            grabberSubsystem.scoreConeCommand(),
                            () -> grabberSubsystem.gamePiece == GamePiece.Cube)
                        .alongWith(new RunCommand(() -> {}, elevatorSubsystem).withTimeout(0.35))
                        .unless(() -> elevatorSubsystem.getExtensionInches() < 10.0)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (elevatorSubsystem.getExtensionInches() > 4.5
        || Constants.ElevatorConstants.PIDController.getGoal().position > 4.5) {
      mode = ExtensionState.EXTEND;
    }
  }

  public static enum ExtensionState {
    RETRACT_AND_ROUTE,
    STORE,
    EXTEND,
  }
}
