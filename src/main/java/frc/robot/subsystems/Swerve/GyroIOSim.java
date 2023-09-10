// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;


public class GyroIOSim implements GyroIO {
  double heading = 0.0;

  @Override
  public GyroIOInputsAutoLogged updateInputs() {
    var inputs = new GyroIOInputsAutoLogged();
    inputs.headingDegrees = heading;
    inputs.rollDegrees = 0.0;
    return inputs;
  }

  @Override
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(heading);
  }

  @Override
  public double getRollDegrees() {
    return 0.0;
  }

  @Override
  public void resetHeading(double degrees) {}
}
