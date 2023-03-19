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

public class SuperstructureSubsystem extends SubsystemBase {
  IntakeSubsystem intakeSubsystem;
  ElevatorSubsystem elevatorSubsystem;
  ArmSubsystem armSubsystem;
  RoutingSubsystem routingSubsystem;
  RollerClawGrabberSubsystem grabberSubsystem;
  SwerveSubsystem swerveSubsystem;
  LEDSubsystem ledSubsystem;

  ExtensionState mode = ExtensionState.RETRACT_AND_ROUTE;

  public Trigger retractAndRouteTrigger = new Trigger(() -> mode == ExtensionState.RETRACT_AND_ROUTE);
  public Trigger storeTrigger = new Trigger(() -> mode == ExtensionState.STORE);
  public Trigger extendTrigger = new Trigger(() -> mode == ExtensionState.EXTEND);

  /** Creates a new SuperstructureSubsystem. */
  public SuperstructureSubsystem(
    IntakeSubsystem intakeSubsystem,
    ElevatorSubsystem elevatorSubsystem,
    ArmSubsystem armSubsystem,
    RoutingSubsystem routingSubsystem,
    RollerClawGrabberSubsystem grabberSubsystem,
    SwerveSubsystem swerveSubsystem,
    LEDSubsystem ledSubsystem
  ) {
    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.routingSubsystem = routingSubsystem;
    this.grabberSubsystem = grabberSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.ledSubsystem = ledSubsystem;
  }

  public SuperstructureSubsystem(IntakeSubsystem intakeSubsystem2, ElevatorSubsystem elevatorSubsystem2,
      RoutingSubsystem routingSubsystem2, GreybotsGrabberSubsystem greybotsGrabberSubsystem,
      SwerveSubsystem swerveSubsystem2, LEDSubsystem ledSubsystem2) {
  }

  public void setMode(ExtensionState mode) {
    this.mode = mode;
  }

  public CommandBase waitExtendToInches(DoubleSupplier extensionInches){
    return new InstantCommand(() -> mode = ExtensionState.EXTEND)
      .andThen(new WaitCommand(0.35))
      .andThen(elevatorSubsystem.extendToInchesCommand(extensionInches))
      .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public CommandBase waitExtendToInches(double extensionInches) {
    return waitExtendToInches(() -> extensionInches);
  }

  public CommandBase waitExtendToGoal(Supplier<ScoringLevels> level) {
    return new PrintCommand("extension target " + swerveSubsystem.getExtension(level.get()))
      .andThen(waitExtendToInches(() -> swerveSubsystem.getExtension(level.get())));
  }

  public CommandBase waitExtendToGoal(ScoringLevels level) {
    return waitExtendToGoal(() -> level);
  }

  public CommandBase waitExtendToGoal() {
    return waitExtendToGoal(swerveSubsystem.getLevel());
  }

  public CommandBase scoreNoAim() {
    return this.waitExtendToGoal(() -> swerveSubsystem.getLevel())
    .deadlineWith(ledSubsystem.setRainbowCommand(), grabberSubsystem.closeCommand())
    .andThen(
      new WaitCommand(0.1),
      new ConditionalCommand(
          grabberSubsystem.susL3Command()
          .raceWith(new RunCommand(() -> {}, elevatorSubsystem))
          .withTimeout(0.5)
          .andThen(new WaitCommand(0.2), grabberSubsystem.openCommand())
          .andThen(new WaitCommand(0.2)), 
          new ConditionalCommand(
            grabberSubsystem.openCommand().andThen(new WaitCommand(0.2)), 
            grabberSubsystem.outakeOpenCommand().withTimeout(0.3), 
            () -> swerveSubsystem.isConeOveride), 
          () -> swerveSubsystem.isConeOveride && swerveSubsystem.getLevel() == ScoringLevels.L3
          )
          .unless(() -> elevatorSubsystem.getExtensionInches() < 10.0)
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (elevatorSubsystem.getExtensionInches() > 4.5 || Constants.ElevatorConstants.PIDController.getGoal().position > 4.5) { // TODO: Find good value for maximum extension before "extended"
      mode = ExtensionState.EXTEND;
    }
    SmartDashboard.putString("Superstructure Mode", mode.toString());

  }

  @Override
  public void simulationPeriodic() {
    elevatorSubsystem.updateMech2d(new Pair<Double,Double>(
      Constants.ElevatorConstants.PIDController.getSetpoint().position, 
      Constants.ArmConstants.PIDController.getSetpoint().position));
  }

  public static enum ExtensionState {
    RETRACT_AND_ROUTE,
    STORE,
    EXTEND,
  }
}
