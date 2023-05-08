// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import frc.lib.components.HighlanderFalcon;
import frc.robot.Constants;

/** Implements the elevator subsystem with falcon 500 motors. */
public class ElevatorIOFalcon implements ElevatorIO {
  private final HighlanderFalcon leader;
  private final HighlanderFalcon follower;

  public ElevatorIOFalcon() {
    leader = new HighlanderFalcon(Constants.ElevatorConstants.elevatorMotorID, 5.45 / 1.0);
    leader.config_kF(0, 0);
    follower = new HighlanderFalcon(Constants.ElevatorConstants.elevatorFollowerID);
    follower.set(ControlMode.Follower, Constants.ElevatorConstants.elevatorMotorID);
    follower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30.0, 30.0, 0.5));
    leader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30.0, 30.0, 0.5));
    leader.configVoltageCompSaturation(10);
    follower.configVoltageCompSaturation(10);
    zeroMotor();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.positionInches = getExtensionInches();
    inputs.percentOut = leader.getMotorOutputPercent();
    inputs.currentAmps = new double[] {leader.getStatorCurrent(), follower.getStatorCurrent()};
  }

  @Override
  public void zeroMotor() {
    leader.setSelectedSensorPosition(0);
  }

  @Override
  public double getExtensionInches() {
    return leader.getRotations() * Constants.ElevatorConstants.elevatorSpoolCircumference * 2.5;
  }

  @Override
  public void setPercentOut(double percentOut, double ff) {
    leader.set(ControlMode.PercentOutput, percentOut, DemandType.ArbitraryFeedForward, ff);
  }

  @Override
  public void setPercentOut(double percentOut) {
    setPercentOut(percentOut, 0.0);
  }

  @Override
  public void stop() {
    setPercentOut(0.0);
  }
}
