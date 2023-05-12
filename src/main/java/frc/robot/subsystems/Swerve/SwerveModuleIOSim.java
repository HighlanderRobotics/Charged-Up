// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveModuleIOSim implements SwerveModuleIO {
  FlywheelSim drivePhysicsSim = new FlywheelSim(DCMotor.getFalcon500(1), 6.75 / 1, 0.025);
  SingleJointedArmSim steerPhysicsSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), 150 / 7, 0.004, Units.inchesToMeters(1.5), 0.0, 2 * Math.PI, false);

  PIDController driveController = new PIDController(Constants.Swerve.simDriveKP, 0, 0);
  SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(Constants.Swerve.simDriveKS, Constants.Swerve.simDriveKV, Constants.Swerve.simDriveKA);

  PIDController steerController = new PIDController(Constants.Swerve.simAngleKP, 0, Constants.Swerve.simAngleKD);

  double velocitySetpoint = 0.0;
  double driveVolts = 0.0;
  double steerSetpoint = 0.0;
  double steerVolts = 0.0;

  int moduleNumber = -1;

  public SwerveModuleIOSim(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    driveVolts = driveController.calculate(drivePhysicsSim.getAngularVelocityRPM(), velocitySetpoint) + driveFeedforward.calculate(velocitySetpoint);
    steerVolts = steerController.calculate(steerPhysicsSim.getAngleRads(), steerSetpoint);

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
  }

}
