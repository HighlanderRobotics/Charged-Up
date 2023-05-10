// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean switchPressed = false;
    public double positionInches = 0.0;
    public double percentOut = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setPercentOut(double percentOut, double ff) {}

  public default void setPercentOut(double percentOut) {}

  public default void stop() {}

  public default void zeroMotor() {}

  public default double getExtensionInches() {
    return 0.0;
  }

  public default boolean getLimitSwitch() {
    return false;
  }
}
