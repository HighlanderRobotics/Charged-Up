// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.components.HighlanderFalcon;
import frc.robot.Constants;

public class RoutingSubsystem extends SubsystemBase {
  HighlanderFalcon routingLeft = new HighlanderFalcon(
    Constants.MechanismConstants.routingLeftID, 
    1.0,
    Constants.MechanismConstants.routingKP, 
    0, 
    0);
  HighlanderFalcon routingRight = new HighlanderFalcon(
    Constants.MechanismConstants.routingRightID, 
    1.0,
    Constants.MechanismConstants.routingKP, 
    0, 
    0);
  // TODO: talk to routing subteam about logic and stuff
  /** Creates a new RoutingSubsystem. */
  public RoutingSubsystem() {
    routingLeft.setNeutralMode(NeutralMode.Brake);
    routingRight.setNeutralMode(NeutralMode.Brake);
  }

  private void run() {
    routingLeft.setPercentOut(0.25); // TODO: find right rpm
    routingRight.setPercentOut(-0.25); // TODO: find right rpm
  }

  private void outake() {
    routingLeft.setPercentOut(-0.15);
    routingRight.setPercentOut(0.15);
  }

  private void stop() {
    routingLeft.setPercentOut(0);
    routingRight.setPercentOut(0);
  }

  public CommandBase runCommand() { // TODO: this is probably the wrong logic, so fix
    return new RunCommand(() -> run(), this).handleInterrupt(() -> stop());
  }

  public CommandBase outakeCommand() {
    return new RunCommand(() -> outake(), this);
  }

  public CommandBase stopCommand() {
    return new RunCommand(() -> stop(), this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
