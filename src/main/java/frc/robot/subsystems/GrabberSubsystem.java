// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.components.HighlanderFalcon;
import frc.lib.components.ReversibleDigitalInput;
import frc.robot.Constants;

public class GrabberSubsystem extends SubsystemBase {
  HighlanderFalcon grabber = new HighlanderFalcon(Constants.MechanismConstants.grabberID);
  DoubleSolenoid solenoid = new DoubleSolenoid(
    PneumaticsModuleType.REVPH, 
    Constants.MechanismConstants.grabberSolenoidFrontID, 
    Constants.MechanismConstants.grabberSolenoidBackID);
  ReversibleDigitalInput limitSwitch = new ReversibleDigitalInput(
    Constants.MechanismConstants.grabberLimitSwitch, 
    Constants.MechanismConstants.isGrabberSwitchReversed);
  /** Creates a new GrabberSubsystem. */
  public GrabberSubsystem() {
    grabber.setNeutralMode(NeutralMode.Brake);
  }

  private void intake() {
    grabber.setPercentOut(-0.7); // TODO: find best value
  }

  private void outake() {
    grabber.setPercentOut(0.5); // TODO: find best value
  }

  private void stop() {
    grabber.setPercentOut(0); // might want to use PID hold
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

  public CommandBase intakeCommand() {
    return new RunCommand(() -> {
        intake();
        open();
    }, this).until(() -> limitSwitch.get())
    .andThen(new RunCommand(() -> {
      stop();
    }, this)).until(() -> !limitSwitch.get()).repeatedly();
  }

  public CommandBase outakeCommand() {
    return new RunCommand(() -> outake(), this);
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

  @Override
 public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("grabber sensor", limitSwitch.get());
  }
}
