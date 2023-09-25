// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Routing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class RoutingSubsystem extends SubsystemBase {
  RoutingIOFalcon io;
  RoutingIOInputsAutoLogged inputs;

  /** Creates a new RoutingSubsystem. */
  public RoutingSubsystem() {
    io = new RoutingIOFalcon();
    inputs = new RoutingIOInputsAutoLogged();
  }

  private void run() {
    io.setPercentOut(0.7);
  }

  private void outake() {
    io.setPercentOut(-0.15);
  }

  private void stop() {
    io.setPercentOut(0.0);
  }

  public CommandBase runCommand() {
    return new RunCommand(() -> run(), this).handleInterrupt(() -> stop());
  }

  public CommandBase outakeCommand() {
    return new RunCommand(() -> outake(), this);
  }

  public CommandBase stopCommand() {
    return new RunCommand(() -> stop(), this);
  }

  public CommandBase slowRunCommand() {
    return new RunCommand(
        () -> {
          io.setPercentOut(-0.1);
        },
        this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    inputs = io.updateInputs();
    Logger.getInstance().processInputs("Routing", inputs);
  }
}
