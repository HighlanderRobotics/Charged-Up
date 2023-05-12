// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

/** Add your docs here. */
public class GyroIOPigeon implements GyroIO {
  Pigeon2 gyro;

  public GyroIOPigeon() {
    gyro = new Pigeon2(Constants.Swerve.pigeonID);
    gyro.configFactoryDefault();
    resetHeading(0.0);
  }

  @Override
  public void updateInputs(GyroIOInputs input) {
    input.headingDegrees = gyro.getYaw();
    input.rollDegrees = gyro.getRoll();
  }

  @Override
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(gyro.getYaw());
  }

  @Override
  public double getRollDegrees() {
    return gyro.getRoll();
  }

  @Override
  public void resetHeading(double degrees) {
    gyro.setYaw(degrees);
  }
}
