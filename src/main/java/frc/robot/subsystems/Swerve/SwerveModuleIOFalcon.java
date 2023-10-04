package frc.robot.subsystems.Swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.components.HighlanderFalcon;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModuleIOFalcon implements SwerveModuleIO {
  private int moduleNumber;
  private Rotation2d angleOffset;

  private TalonFX steerMotor;
  private TalonFX driveMotor;
  private CANCoder angleEncoder;

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  public SwerveModuleIOFalcon(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANCoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    steerMotor = new HighlanderFalcon(moduleConstants.angleMotorID);
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new HighlanderFalcon(moduleConstants.driveMotorID);
    configDriveMotor();
  }

  @Override
  public SwerveModuleIOInputsAutoLogged updateInputs() {
    var inputs = new SwerveModuleIOInputsAutoLogged();
    inputs.moduleNumber = moduleNumber;

    inputs.drivePositionRotations = driveMotor.getSelectedSensorPosition() / 2048;
    inputs.driveSpeedRPS = driveMotor.getSelectedSensorVelocity() * 10 / 2048;
    inputs.drivePercentOut = driveMotor.getMotorOutputPercent();
    inputs.driveCurrentAmps = driveMotor.getStatorCurrent();
    inputs.driveTemparature = driveMotor.getTemperature();

    inputs.absoluteEncoderRotations = getAbsoluteRotation().getRadians();

    inputs.steerPositionRotations = steerMotor.getSelectedSensorPosition() / 2048;
    inputs.steerSpeedRPS = steerMotor.getSelectedSensorVelocity() * 10 / 2048;
    inputs.steerPercentOut = steerMotor.getMotorOutputPercent();
    inputs.steerCurrentAmps = steerMotor.getStatorCurrent();
    inputs.steerTemparature = steerMotor.getTemperature();
    return inputs;
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
    desiredState = CTREModuleState.optimize(desiredState, getState().angle);
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.set(ControlMode.PercentOutput, percentOutput);
    } else {
      double velocity =
          Conversions.MPSToFalcon(
              desiredState.speedMetersPerSecond,
              Constants.Swerve.wheelCircumference,
              Constants.Swerve.driveGearRatio);
      driveMotor.set(
          ControlMode.Velocity,
          velocity,
          DemandType.ArbitraryFeedForward,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents Jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
            ? getAngle()
            : desiredState.angle;

    steerMotor.set(
        ControlMode.Position,
        Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio));
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(
        Conversions.falconToDegrees(
            steerMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
  }

  private Rotation2d getAbsoluteRotation() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  @Override
  public void resetToAbsolute() {
    double absolutePosition =
        Conversions.degreesToFalcon(
            getAbsoluteRotation().getDegrees() - angleOffset.getDegrees(),
            Constants.Swerve.angleGearRatio);
    steerMotor.setSelectedSensorPosition(absolutePosition);
  }

  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
  }

  private void configAngleMotor() {
    steerMotor.configFactoryDefault();
    steerMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
    steerMotor.setInverted(Constants.Swerve.angleMotorInvert);
    steerMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
    resetToAbsolute();
  }

  private void configDriveMotor() {
    driveMotor.configFactoryDefault();
    driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
    driveMotor.setInverted(Constants.Swerve.driveMotorInvert);
    driveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
    driveMotor.setSelectedSensorPosition(0);
  }

  private SwerveModuleState getState() {
    return new SwerveModuleState(
        Conversions.falconToMPS(
            driveMotor.getSelectedSensorVelocity(),
            Constants.Swerve.wheelCircumference,
            Constants.Swerve.driveGearRatio),
        getAngle());
  }
}
