// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;

public class SwerveModuleIOSim implements SwerveModuleIO {
  // Physics sims arent in use right now, but could add them in later
  FlywheelSim drivePhysicsSim = new FlywheelSim(DCMotor.getFalcon500(1), 6.75 / 1, 0.025);
  SingleJointedArmSim steerPhysicsSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(1),
          150 / 7,
          0.004,
          Units.inchesToMeters(1.5),
          0.0,
          2 * Math.PI,
          false);

  PIDController driveController = new PIDController(Constants.Swerve.simDriveKP, 0, 0);
  SimpleMotorFeedforward driveFeedforward =
      new SimpleMotorFeedforward(
          Constants.Swerve.simDriveKS, Constants.Swerve.simDriveKV, Constants.Swerve.simDriveKA);

  PIDController steerController =
      new PIDController(Constants.Swerve.simAngleKP, 0, Constants.Swerve.simAngleKD);

  double velocitySetpoint = 0.0;
  double drivePosition = 0.0;
  double driveVolts = 0.0;
  double steerSetpoint = 0.0;
  double steerVolts = 0.0;

  int moduleNumber = -1;

  public SwerveModuleIOSim(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
  }

  @Override
  public SwerveModuleIOInputsAutoLogged updateInputs() {
    var inputs = new SwerveModuleIOInputsAutoLogged();

    // Update physics "sim"
    drivePosition += velocitySetpoint * 0.020;
    // Update contorl loops
    driveVolts = 0.0;
    steerVolts = 0.0;

    // Update logs
    inputs.moduleNumber = moduleNumber;

    inputs.drivePositionRotations = 0.0;
    inputs.driveSpeedRPS = velocitySetpoint;
    inputs.drivePercentOut = driveVolts / 12.0;
    inputs.driveCurrentAmps = 0.0;
    inputs.driveTemparature = 0.0;

    inputs.absoluteEncoderRotations = getAbsoluteRotation().getRadians();

    inputs.steerPositionRotations = steerSetpoint;
    inputs.steerSpeedRPS = 0.0;
    inputs.steerPercentOut = steerVolts / 12.0;
    inputs.steerCurrentAmps = 0.0;
    inputs.steerTemparature = 0.0;

    return inputs;
  }

  @Override
  public void setDesiredState(SwerveModuleState state, boolean isOpenLoop) {
    velocitySetpoint = state.speedMetersPerSecond;
    steerSetpoint = state.angle.getRadians();
  }

  @Override
  public Rotation2d getAbsoluteRotation() {
    return Rotation2d.fromRadians(steerSetpoint);
  }

  @Override
  public void resetToAbsolute() {}

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(velocitySetpoint, getAbsoluteRotation());
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(drivePosition, getAbsoluteRotation());
  }

  @Override
  public int getModuleNumber() {
    return moduleNumber;
  }

  private double rpmToMetersPerSecond(double rpm) {
    return rpm * (1.0 / 60.0) * 2 * Math.PI * Units.inchesToMeters(4.0);
  }
}
