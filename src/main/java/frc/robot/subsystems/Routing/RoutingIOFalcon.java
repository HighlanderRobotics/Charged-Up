// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Routing;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.lib.components.HighlanderFalcon;
import frc.robot.Constants;

/** Add your docs here. */
public class RoutingIOFalcon implements RoutingIO {
  HighlanderFalcon left =
      new HighlanderFalcon(
          Constants.MechanismConstants.routingLeftID,
          1.0,
          Constants.MechanismConstants.routingKP,
          0,
          0);
  HighlanderFalcon right =
      new HighlanderFalcon(
          Constants.MechanismConstants.routingRightID,
          1.0,
          Constants.MechanismConstants.routingKP,
          0,
          0);

  public RoutingIOFalcon() {
    left.setNeutralMode(NeutralMode.Brake);
    right.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void updateInputs(RoutingIOInputs input) {
    input.leftPercentOut = left.getMotorOutputPercent();
    input.speedLeftRPS = left.getSelectedSensorVelocity() * 10.0 / 2048;
    input.leftCurrentAmps = left.getStatorCurrent();
    input.rightPercentOut = right.getMotorOutputPercent();
    input.speedRightRPS = right.getSelectedSensorVelocity() * 10.0 / 2048;
    input.rightCurrentAmps = right.getStatorCurrent();
  }

  @Override
  public void setPercentOut(double percentOut) {
    left.setPercentOut(percentOut);
    right.setPercentOut(-percentOut);
  }
}
