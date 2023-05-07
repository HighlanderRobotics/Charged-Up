package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.components.HighlanderFalcon;
import frc.lib.logging.LoggingWrapper;
import frc.robot.Constants;

@Deprecated
public class ArmSubsystem extends SubsystemBase {
  HighlanderFalcon armMotor;
  boolean enabled = true;
  DutyCycleEncoder absEncoder;

  public ArmSubsystem() {
    armMotor = new HighlanderFalcon(Constants.ArmConstants.armMotorID, "CANivore");
    armMotor.setNeutralMode(NeutralMode.Brake);
    armMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30.0, 15.0, 0.5));
    absEncoder = new DutyCycleEncoder(Constants.ArmConstants.armEncoderID);
    setGoal(-1.2);
  }

  private void updatePID(double output, TrapezoidProfile.State state) {
    if (getMeasurement() > Constants.ArmConstants.armMaximumAngle && output < 0) {
      armMotor.setPercentOut(0);
    } else if (getMeasurement() < Constants.ArmConstants.armMinimumAngle && output > 0) {
      armMotor.setPercentOut(0);
    } else {
      armMotor.set(ControlMode.PercentOutput, MathUtil.clamp(output, -0.75, 0.75));
    }
  }

  /** 0 is down, PI/2 is horizontal */
  public void setGoal(double position) {
    // double clampedPosition = MathUtil.clamp(position, -0.6, -1.5);
    Constants.ArmConstants.PIDController.reset(getRotation().getRadians());
    Constants.ArmConstants.PIDController.setGoal(position);
  }

  public CommandBase runToRotationCommand(double radians) {
    return new InstantCommand(() -> setGoal(radians), this)
        .andThen(new WaitUntilCommand(() -> isAtSetpoint()));
  }

  public CommandBase runToHorizontalCommand() {
    return runToRotationCommand(-1.7);
  }

  public CommandBase runToRoutingCommand() {
    return runToRotationCommand(-1.4);
  }

  public void jogUp() {
    armMotor.setPercentOut(0.1);
  }

  public void jogDown() {
    armMotor.setPercentOut(-0.1);
  }

  public void stop() {
    armMotor.setPercentOut(0);
  }

  private double getMeasurement() {
    return (absEncoder.getAbsolutePosition() - Constants.ArmConstants.armOffset) % 1.0;
  }

  public Rotation2d getRotation() {
    return new Rotation2d((getMeasurement()) * Math.PI * 2).minus(new Rotation2d());
  }

  public boolean isAtSetpoint() {
    return Constants.ArmConstants.PIDController.atGoal();
  }

  public void enable() {
    enabled = true;
  }

  public void disable() {
    enabled = false;
  }

  @Override
  public void periodic() {
    double pidOut = Constants.ArmConstants.PIDController.calculate(getRotation().getRadians());
    LoggingWrapper.shared.add("arm pidout", pidOut);
    if (enabled) {
      updatePID(pidOut, Constants.ArmConstants.PIDController.getSetpoint());
    }

    LoggingWrapper.shared.add("arm radians", getRotation().getRadians());
    LoggingWrapper.shared.add("arm encoder native", getMeasurement());
    LoggingWrapper.shared.add("arm current", armMotor.getSupplyCurrent());
    LoggingWrapper.shared.add("arm goal", Constants.ArmConstants.PIDController.getGoal().position);
    LoggingWrapper.shared.add(
        "arm setpoint", Constants.ArmConstants.PIDController.getSetpoint().position);
  }
}
