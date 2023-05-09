// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Routing;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.lib.components.HighlanderFalcon;
import frc.robot.Constants;

/** Add your docs here. */
public class RoutingIOFalcon implements RoutingIO {
  HighlanderFalcon routingLeft =
      new HighlanderFalcon(
          Constants.MechanismConstants.routingLeftID,
          1.0,
          Constants.MechanismConstants.routingKP,
          0,
          0);
  HighlanderFalcon routingRight =
      new HighlanderFalcon(
          Constants.MechanismConstants.routingRightID,
          1.0,
          Constants.MechanismConstants.routingKP,
          0,
          0);

  public RoutingIOFalcon() {
    routingLeft.setNeutralMode(NeutralMode.Brake);
    routingRight.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void updateInputs(RoutingIOInputs input) {
    input.routingLeftPercentOut = routingLeft.getMotorOutputPercent();
    input.routingLeftRPS = routingLeft.getSelectedSensorVelocity() * 10.0 / 2048;
    input.routingRightPercentOut = routingRight.getMotorOutputPercent();
    input.routingRightRPS = routingRight.getSelectedSensorVelocity() * 10.0 / 2048;
    input.intakeCurrentAmps =
        new double[] {routingLeft.getStatorCurrent(), routingRight.getStatorCurrent()};
  }

  @Override
  public void setPercentOut(double percentOut) {
    routingLeft.setPercentOut(percentOut);
    routingRight.setPercentOut(-percentOut);
  }
}
