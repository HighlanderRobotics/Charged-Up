// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
  @AutoLog
  public static class SwerveModuleIOInputs {
    public long moduleNumber = -1;

    public double drivePositionRotations = 0.0;
    public double driveSpeedRPS = 0.0;
    public double drivePercentOut = 0.0;
    public double driveCurrentAmps = 0.0;
    public double driveTemparature = 0.0;

    public double absoluteEncoderRotations = 0.0;

    public double steerPositionRotations = 0.0;
    public double steerSpeedRPS = 0.0;
    public double steerPercentOut = 0.0;
    public double steerCurrentAmps = 0.0;
    public double steerTemparature = 0.0;
  }

  public abstract SwerveModuleIOInputsAutoLogged updateInputs();

  public abstract void setDesiredState(SwerveModuleState state, boolean isOpenLoop);

  public abstract Rotation2d getAbsoluteRotation();

  public abstract void resetToAbsolute();

  public abstract SwerveModuleState getState();

  public abstract SwerveModulePosition getPosition();

  public abstract int getModuleNumber();
}
