// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {}
  public void extend() {}
  public void place() {}
  public void retract() {}
  public boolean isAtSetpoint() { return false; }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
