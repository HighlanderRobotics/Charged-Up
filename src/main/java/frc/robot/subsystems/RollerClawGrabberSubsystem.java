// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.components.HighlanderFalcon;
import frc.lib.components.ReversibleDigitalInput;
import frc.lib.logging.LoggingWrapper;
import frc.robot.Constants;

@Deprecated
public class RollerClawGrabberSubsystem extends SubsystemBase {
  HighlanderFalcon grabber = new HighlanderFalcon(Constants.MechanismConstants.grabberIntakeID, "CANivore", 1, 5e-1, 0.0, 0.0);
  DoubleSolenoid solenoid = new DoubleSolenoid(
    PneumaticsModuleType.REVPH, 
    Constants.MechanismConstants.grabberSolenoidFrontID, 
    Constants.MechanismConstants.grabberSolenoidBackID);
  ReversibleDigitalInput limitSwitch = new ReversibleDigitalInput(
    Constants.MechanismConstants.grabberLimitSwitch, 
    false);
  /** Creates a new GrabberSubsystem. */
  public RollerClawGrabberSubsystem() {
    grabber.setNeutralMode(NeutralMode.Brake);
    grabber.configVoltageCompSaturation(10);
  }

  private void intake() {
    grabber.setPercentOut(-1); // TODO: find best value
  }

  private void outake() {
    grabber.set(ControlMode.PercentOutput, 0.17); // TODO: find best value
  }

  private void stop() {
    // grabber.set(ControlMode.Velocity, 0); // might want to use PID hold
    grabber.setPercentOut(0);
  }

  private void open() {
    solenoid.set(Value.kReverse);
  }

  private void close() {
    solenoid.set(Value.kForward);
  }

  private void neutral() {
    solenoid.set(Value.kOff);
  }

  public CommandBase intakeOpenCommand() {
    return new RunCommand(() -> {
        intake();
        open();
    }, this)
    .raceWith(
      new WaitUntilCommand(() -> limitSwitch.get())
      .andThen(new WaitCommand(0.25)))
    .andThen(new RunCommand(() -> {
      stop();
    }, this)).until(() -> !limitSwitch.get()).repeatedly();
  }

  public CommandBase intakeClosedCommand() {
    return new RunCommand(() -> {
        intake();
        close();
    }, this)
    .raceWith(
      new WaitUntilCommand(() -> limitSwitch.get())
      .andThen(new WaitCommand(0.25)))
    .andThen(new RunCommand(() -> {
      stop();
    }, this)).until(() -> !limitSwitch.get()).repeatedly();
  }

  public CommandBase intakeCommand() {
    return new RunCommand(() -> {
        intake();
    }, this)
    .raceWith(
      new WaitUntilCommand(() -> limitSwitch.get())
      .andThen(new WaitCommand(0.25)))
    .andThen(new RunCommand(() -> {
      stop();
    }, this)).until(() -> !limitSwitch.get()).repeatedly();
  }

  public CommandBase outakeCommand() {
    return new RunCommand(() -> outake(), this);
  }

  public CommandBase outakeNeutralCommand() {
    return new InstantCommand(() -> outake(), this)
      .andThen(new WaitCommand(0.25),
      new RunCommand(() -> {outake(); neutral();}, this));
  }

  public CommandBase outakeOpenCommand() {
    return new RunCommand(() -> {grabber.setPercentOut(0.4); open();}, this);
  }

  public CommandBase openCommand() {
    return new InstantCommand(() -> open(), this);
  }

   public CommandBase closeCommand() {
     return new InstantCommand(() -> close(), this);
  }

  public CommandBase neutralCommand() {
    return new InstantCommand(() -> neutral(), this);
  }

  public CommandBase stopCommand() {
    return new RunCommand(() -> stop(), this);
  }

  public boolean hasGamePiece() {
    return limitSwitch.get();
  }

  public CommandBase stopAndClose(){
    return new RunCommand(() -> {stop(); close();}, this);
  }

  public CommandBase susL3Command() {
    return new InstantCommand(() -> {grabber.setSelectedSensorPosition(0); neutral();})
      .andThen(new RunCommand(() -> grabber.set(ControlMode.Position, 2.0 * 2048), this)
      // .until(() -> grabber.getClosedLoopError() < 200)
      .andThen(openCommand(), stopCommand()));
  }

  @Override
 public void periodic() {
    // This method will be called once per scheduler run
    LoggingWrapper.shared.add("grabber sensor", limitSwitch.get());
    LoggingWrapper.shared.add("grabber output", grabber.getMotorOutputPercent());
    LoggingWrapper.shared.add("grabber vel falcon units", grabber.getSelectedSensorVelocity());
  }
}
