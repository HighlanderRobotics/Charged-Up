// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Grabber;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface GrabberIO {
  @AutoLog
  public static class GrabberIOInputs {
    public double rollersSpeedRPS = 0.0;
    public double rollersPercentOut = 0.0;
    public double rollersCurrentAmps = 0.0;
    public double pivotPositionTicks = 0.0;
    public double pivotPercentOut = 0.0;
    public double pivotCurrentAmps = 0.0;
    public boolean switchPressed = false;
    public boolean beambreakTriggered = false;
  }

  public default void updateInputs(GrabberIOInputs inputs) {}

  public default void setRollersPercentOut(double percentOut) {}

  public default void setPivotTarget(double encoderTicks) {}

  public default void setPivotPercentOut(double percentOut) {}

  public default void resetPivotEncoder(int newPosition) {}

  public default void resetPivotEncoder() {}

  public default double getPivotPosition() {
    return 0.0;
  }

  public default double getPivotError() {
    return 0.0;
  }

  public default double getRollersError() {
    return 0.0;
  }

  public default boolean getLimitSwitch() {
    return false;
  }

  public default boolean getBeambreak() {
    return false;
  }
}
