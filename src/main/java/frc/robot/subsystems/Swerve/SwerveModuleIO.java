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
        (driveSpeedRPS / Constants.Swerve.driveGearRatio) * Math.PI * Units.inchesToMeters(4),
        Rotation2d.fromRotations((steerPositionRotations / Constants.Swerve.angleGearRatio))
      );
    }

    public SwerveModulePosition getPosition() {
      return new SwerveModulePosition(
        (drivePositionRotations / Constants.Swerve.driveGearRatio) * Math.PI * Units.inchesToMeters(4),
        Rotation2d.fromRotations((steerPositionRotations / Constants.Swerve.angleGearRatio))
      );
    }

    public Rotation2d getOffset() {
      switch ((int) moduleNumber) {
        case 0: {
          return Constants.Swerve.Mod0.angleOffset;
        }
        case 1: {
          return Constants.Swerve.Mod1.angleOffset;
        }
        case 2: {
          return Constants.Swerve.Mod2.angleOffset;
        }
        case 3: {
          return Constants.Swerve.Mod3.angleOffset;
        }
      }
      return new Rotation2d();
    }
  }

  public abstract SwerveModuleIOInputsAutoLogged updateInputs();

  public abstract void setDesiredState(SwerveModuleState state, boolean isOpenLoop);

  public abstract void resetToAbsolute();
}
