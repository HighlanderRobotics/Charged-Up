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

  public abstract void updateInputs(GrabberIOInputs inputs);

  public abstract void setRollersPercentOut(double percentOut);

  public abstract void setPivotTarget(double encoderTicks);

  public abstract void setPivotPercentOut(double percentOut);

  public abstract void resetPivotEncoder(int newPosition);

  public abstract void resetPivotEncoder();

  public abstract double getPivotPosition();

  public abstract double getPivotError();

  public abstract double getRollersError();

  public abstract boolean getLimitSwitch();

  public abstract boolean getBeambreak();
}
