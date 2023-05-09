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
    public double intakeRPS = 0.0;
    public double intakePercentOut = 0.0;
    public double intakeCurrentAmps = 0.0;
  }

  public default void updateInputs(IntakeIOInputs input) {}

  public default void extend() {}

  public default void retract() {}

  public default void setPercentOut(double percentOut) {}
}
