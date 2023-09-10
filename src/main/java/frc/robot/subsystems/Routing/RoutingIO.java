// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Routing;

import org.littletonrobotics.junction.AutoLog;


public interface RoutingIO {
  @AutoLog
  public static class RoutingIOInputs {
    public double speedLeftRPS = 0.0;
    public double leftPercentOut = 0.0;
    public double leftCurrentAmps = 0.0;
    public double speedRightRPS = 0.0;
    public double rightPercentOut = 0.0;
    public double rightCurrentAmps = 0.0;
  }

  public abstract RoutingIOInputsAutoLogged updateInputs();

  public abstract void setPercentOut(double percentOut);
}
