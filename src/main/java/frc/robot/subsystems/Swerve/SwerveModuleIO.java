// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
  @AutoLog
  public static class SwerveModuleIOInputs {
    public long moduleNumber = 0;

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

    public SwerveModuleState getState() {
      return new SwerveModuleState(
        (driveSpeedRPS / Constants.Swerve.driveGearRatio) * 2 * Math.PI * Units.inchesToMeters(2),
        Rotation2d.fromRotations((steerPositionRotations / Constants.Swerve.angleGearRatio))
      );
    }

    public SwerveModulePosition getPosition() {
      return new SwerveModulePosition(
        (drivePositionRotations / Constants.Swerve.driveGearRatio) * 2 * Math.PI * Units.inchesToMeters(2),
        Rotation2d.fromRotations((steerPositionRotations / Constants.Swerve.angleGearRatio))
      );
    }
  }

  public abstract SwerveModuleIOInputsAutoLogged updateInputs();

  public abstract void setDesiredState(SwerveModuleState state, boolean isOpenLoop);

  public abstract void resetToAbsolute();
}
