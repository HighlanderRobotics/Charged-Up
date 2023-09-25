// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean switchPressed = false;
    public double positionInches = 0.0;
    public double percentOut = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public abstract ElevatorIOInputsAutoLogged updateInputs();

  public abstract void setPercentOut(double percentOut, double ff);

  public abstract void setPercentOut(double percentOut);

  public abstract void stop();

  public abstract void zeroMotor();

  public abstract double getExtensionInches();

  public abstract boolean getLimitSwitch();
}
