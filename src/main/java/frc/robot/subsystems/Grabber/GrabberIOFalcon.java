// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Grabber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import frc.lib.components.HighlanderFalcon;
import frc.lib.components.ReversibleDigitalInput;
import frc.robot.Constants;

/** Add your docs here. */
public class GrabberIOFalcon implements GrabberIO {
  HighlanderFalcon rollers =
      new HighlanderFalcon(
          Constants.MechanismConstants.grabberIntakeID, "CANivore", 1, 5e-1, 0.0, 0.0);
  HighlanderFalcon pivot =
      new HighlanderFalcon(
          Constants.MechanismConstants.grabberPivotID, "CANivore", 1.0, 2.0e-2, 0.0, 0.0);
  ReversibleDigitalInput resetLimitSwitch =
      new ReversibleDigitalInput(Constants.MechanismConstants.grabberLimitSwitch, true);
  ReversibleDigitalInput cubeBeambreak =
      new ReversibleDigitalInput(Constants.MechanismConstants.grabberBeambreak, true);

  public GrabberIOFalcon() {
    rollers.configVoltageCompSaturation(10);
    rollers.setNeutralMode(NeutralMode.Brake);
    pivot.setNeutralMode(NeutralMode.Brake);
    pivot.config_kF(0, 0);
    pivot.setInverted(TalonFXInvertType.CounterClockwise);
  }

  @Override
  public void updateInputs(GrabberIOInputs inputs) {
    inputs.beambreakTriggered = cubeBeambreak.get();
    inputs.switchPressed = resetLimitSwitch.get();

    inputs.rollersPercentOut = rollers.getMotorOutputPercent();
    inputs.rollersSpeedRPS = rollers.getSelectedSensorVelocity() * 10 / 2048;
    inputs.rollersCurrentAmps = rollers.getStatorCurrent();

    inputs.pivotPercentOut = rollers.getMotorOutputPercent();
    inputs.pivotPositionTicks = rollers.getSelectedSensorPosition();
    inputs.pivotCurrentAmps = rollers.getStatorCurrent();
  }

  @Override
  public void setRollersPercentOut(double percentOut) {
    rollers.setPercentOut(percentOut);
  }

  @Override
  public void setPivotTarget(double encoderTicks) {
    pivot.set(ControlMode.Position, encoderTicks);
  }

  @Override
  public void setPivotPercentOut(double percentOut) {
    System.out.println("percent out: "+ percentOut);
    pivot.setPercentOut(percentOut);
  }

  @Override
  public void resetPivotEncoder(int newPosition) {
    pivot.setSelectedSensorPosition(newPosition);
  }

  @Override
  public void resetPivotEncoder() {
    resetPivotEncoder(0);
  }

  @Override
  public double getPivotPosition() {
    return pivot.getSelectedSensorPosition();
  }

  @Override
  public double getPivotError() {
    return pivot.getClosedLoopError();
  }

  // TODO check what happens when we remove this with robot
  /** this shouldnt need to exist */
  @Override
  public double getRollersError() {
    return rollers.getClosedLoopError();
  }

  @Override
  public boolean getLimitSwitch() {
    return resetLimitSwitch.get();
  }

  @Override
  public boolean getBeambreak() {
    return cubeBeambreak.get();
  }
}
