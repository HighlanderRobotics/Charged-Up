// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean isExtended = false;
    public double speedRPS = 0.0;
    public double percentOut = 0.0;
    public double currentAmps = 0.0;
  }

  public abstract void updateInputs(IntakeIOInputs input);

  public abstract void extend();

  public abstract void retract();

  public abstract void setPercentOut(double percentOut);
}
