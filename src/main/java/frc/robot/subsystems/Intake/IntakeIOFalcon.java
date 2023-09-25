// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.lib.components.HighlanderFalcon;
import frc.robot.Constants;

public class IntakeIOFalcon implements IntakeIO {
  private final HighlanderFalcon motor =
      new HighlanderFalcon(Constants.MechanismConstants.intakeID, 1.0, 0.15, 0, 0);
  private final DoubleSolenoid piston =
      new DoubleSolenoid(
          PneumaticsModuleType.REVPH,
          Constants.MechanismConstants.intakeSolenoidForwardID,
          Constants.MechanismConstants.intakeSolenoidBackwardID);
  private boolean isExtended = false;

  public IntakeIOFalcon() {}

  @Override
  public IntakeIOInputsAutoLogged updateInputs() {
    var inputs = new IntakeIOInputsAutoLogged();
    inputs.isExtended = isExtended;
    inputs.percentOut = motor.getMotorOutputPercent();
    inputs.speedRPS = motor.getSelectedSensorVelocity() * 10 / 2048;
    inputs.currentAmps = motor.getStatorCurrent();
    return inputs;
  }

  @Override
  public void setPercentOut(double percentOut) {
    motor.setPercentOut(percentOut);
  }

  @Override
  public void extend() {
    piston.set(Value.kForward);
    isExtended = true;
  }

  @Override
  public void retract() {
    piston.set(Value.kReverse);
    isExtended = false;
  }
}
