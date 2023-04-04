// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem.ScoringLevels;
import frc.robot.subsystems.GreybotsGrabberSubsystem.GamePiece;

public class SuperstructureSubsystem extends SubsystemBase {
  IntakeSubsystem intakeSubsystem;
  ElevatorSubsystem elevatorSubsystem;
  RoutingSubsystem routingSubsystem;
  SwerveSubsystem swerveSubsystem;
  GreybotsGrabberSubsystem greybotsGrabberSubsystem;
  LEDSubsystem ledSubsystem;

  ExtensionState mode = ExtensionState.RETRACT_AND_ROUTE;

  public Trigger retractAndRouteTrigger = new Trigger(() -> mode == ExtensionState.RETRACT_AND_ROUTE);
  public Trigger storeTrigger = new Trigger(() -> mode == ExtensionState.STORE);
  public Trigger extendTrigger = new Trigger(() -> mode == ExtensionState.EXTEND);

  /** Creates a new SuperstructureSubsystem. */
  public SuperstructureSubsystem(
    IntakeSubsystem intakeSubsystem,
    ElevatorSubsystem elevatorSubsystem,
    RoutingSubsystem routingSubsystem,
    SwerveSubsystem swerveSubsystem,
    GreybotsGrabberSubsystem greybotsGrabberSubsystem,
    LEDSubsystem ledSubsystem
  ) {
    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.routingSubsystem = routingSubsystem;
    this.greybotsGrabberSubsystem = greybotsGrabberSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.ledSubsystem = ledSubsystem;
  }

  public void setMode(ExtensionState mode) {
    this.mode = mode;
  }

  public CommandBase waitExtendToInches(DoubleSupplier extensionInches){
    return intakeSubsystem.extendCommand()//new InstantCommand(() -> mode = ExtensionState.EXTEND)
      .andThen(new WaitCommand(0.35))
      .andThen(elevatorSubsystem.extendToInchesCommand(extensionInches))
      .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public CommandBase waitExtendToInches(double extensionInches) {
    return waitExtendToInches(() -> extensionInches);
  }

  public CommandBase waitExtendToGoal(Supplier<ScoringLevels> level) {
    return new PrintCommand("extension target " + swerveSubsystem.getExtension(level.get(), greybotsGrabberSubsystem.gamePiece == GamePiece.Cone))
      .andThen(waitExtendToInches(() -> swerveSubsystem.getExtension(level.get(), greybotsGrabberSubsystem.gamePiece == GamePiece.Cone)));
  }

  public CommandBase waitExtendToGoal(ScoringLevels level) {
    return waitExtendToGoal(() -> level);
  }

  public CommandBase waitExtendToGoal() {
    return waitExtendToGoal(swerveSubsystem.getLevel());
  }

  public CommandBase scoreNoAim() {
    return routingSubsystem.stopCommand().raceWith(this.waitExtendToGoal(() -> swerveSubsystem.getLevel())
      .deadlineWith(ledSubsystem.setRainbowCommand(), 
        new WaitCommand(0.4)
          .andThen(greybotsGrabberSubsystem.stopCommand().withTimeout(0.1), greybotsGrabberSubsystem.runToScoringCommand()))
      .withTimeout(1.0)
    .andThen(new WaitCommand(.5),
      new ConditionalCommand(
      greybotsGrabberSubsystem.scoreCubeCommand(),
      greybotsGrabberSubsystem.scoreConeCommand(),
      () -> greybotsGrabberSubsystem.gamePiece == GamePiece.Cube
    ).raceWith(new RunCommand(() -> {}, elevatorSubsystem))
    .unless(() -> elevatorSubsystem.getExtensionInches() < 10.0)//,
    // new WaitCommand(0.25)
    // elevatorSubsystem.extendToInchesCommand(1.0)
    ));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (elevatorSubsystem.getExtensionInches() > 4.5 || Constants.ElevatorConstants.PIDController.getGoal().position > 4.5) { // TODO: Find good value for maximum extension before "extended"
      mode = ExtensionState.EXTEND;
    }
    SmartDashboard.putString("Superstructure Mode", mode.toString());

  }

  public static enum ExtensionState {
    RETRACT_AND_ROUTE,
    STORE,
    EXTEND,
  }
}
