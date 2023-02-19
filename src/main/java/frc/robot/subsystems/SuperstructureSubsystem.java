// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class SuperstructureSubsystem extends SubsystemBase {
  IntakeSubsystem intakeSubsystem;
  ElevatorSubsystem elevatorSubsystem;
  ArmSubsystem armSubsystem;
  RoutingSubsystem routingSubsystem;
  GrabberSubsystem grabberSubsystem;

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
    GrabberSubsystem grabberSubsystem
  ) {
    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.routingSubsystem = routingSubsystem;
    this.grabberSubsystem = grabberSubsystem;
  }

  public void setMode(ExtensionState mode) {
    this.mode = mode;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (ElevatorSubsystem.solveForwardKinematics(
          armSubsystem.getRotation().getRadians(), 
          elevatorSubsystem.getExtensionInches()
        ).getX() > 6 || ElevatorSubsystem.solveForwardKinematics(
          Constants.RotatingArmConstants.PIDController.getGoal().position, 
          Constants.ElevatorConstants.PIDController.getGoal().position
        ).getX() > 6 ) { // TODO: Find good value for maximum extension before "extended"
      mode = ExtensionState.EXTEND;
    }
  }

  public static enum ExtensionState {
    RETRACT_AND_ROUTE,
    STORE,
    EXTEND,
  }
}
