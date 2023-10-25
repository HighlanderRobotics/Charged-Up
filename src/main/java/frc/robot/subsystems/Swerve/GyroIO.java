// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public double headingDegrees;
    public double rollDegrees;
  }

  public abstract GyroIOInputsAutoLogged updateInputs();

  public abstract void resetHeading(double degrees);
}
