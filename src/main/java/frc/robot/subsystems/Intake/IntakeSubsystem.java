// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeIO.IntakeIOInputs;

public class IntakeSubsystem extends SubsystemBase {
  IntakeIOFalcon io;
  IntakeIOInputs inputs;
  // Timer to make sure that the intake has time to extend when we check if its out
  Timer timeSinceExtended = new Timer();
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    io = new IntakeIOFalcon();
    inputs = new IntakeIOInputs();
    timeSinceExtended.start();
  }

  private void run() {
    io.setPercentOut(0.5);
  }

  private void outake() {
    io.setPercentOut(-0.4);
  }

  private void stop() {
    io.setPercentOut(0);
  }

  private void extend() {
    io.extend();
    timeSinceExtended.reset();
  }

  private void retract() {
    io.retract();
  }

  public CommandBase runCommand() {
    return new InstantCommand(
            () -> {
              this.run();
              this.extend();
            },
            this)
        .andThen(new RunCommand(() -> {}, this));
  }

  public CommandBase stopCommand() {
    return new InstantCommand(
        () -> {
          this.stop();
          this.retract();
        },
        this);
  }

  public CommandBase extendCommand() {
    return new InstantCommand(
        () -> {
          this.extend();
          stop();
        },
        this);
  }

  public CommandBase outakeCommand() {
    return new RunCommand(
        () -> {
          outake();
          extend();
        },
        this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
  }

  @Override
  public void simulationPeriodic() {}
}
