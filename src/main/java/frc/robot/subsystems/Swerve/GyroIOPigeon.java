// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class GyroIOPigeon implements GyroIO {
  Pigeon2 gyro;

  public GyroIOPigeon() {
    gyro = new Pigeon2(Constants.Swerve.pigeonID);
    gyro.configFactoryDefault();
    resetHeading(0.0);
  }

  @Override
  public GyroIOInputsAutoLogged updateInputs() {
    var inputs = new GyroIOInputsAutoLogged();
    inputs.headingDegrees = gyro.getYaw();
    inputs.rollDegrees = gyro.getRoll();
    return inputs;
  }

  @Override
  public void resetHeading(double degrees) {
    gyro.setYaw(degrees);
  }
}
