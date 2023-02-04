// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.components.HighlanderFalcon;
import frc.lib.components.ReversibleDigitalInput;
import frc.robot.Constants;

public class GrabberSubsystem extends SubsystemBase {
  HighlanderFalcon grabber = new HighlanderFalcon(Constants.MechanismConstants.grabberID);
  DoubleSolenoid solenoidTop = new DoubleSolenoid(
    PneumaticsModuleType.REVPH, 
    Constants.MechanismConstants.grabberSolenoidTopFrontID, 
    Constants.MechanismConstants.grabberSolenoidTopBackID);
  DoubleSolenoid solenoidBottom = new DoubleSolenoid(
    PneumaticsModuleType.REVPH, 
    Constants.MechanismConstants.grabberSolenoidBottomFrontID, 
    Constants.MechanismConstants.grabberSolenoidBottomBackID);
  ReversibleDigitalInput limitSwitch = new ReversibleDigitalInput(
    Constants.MechanismConstants.grabberLimitSwitch, 
    Constants.MechanismConstants.isGrabberSwitchReversed);
  /** Creates a new GrabberSubsystem. */
  public GrabberSubsystem() {
    grabber.setNeutralMode(NeutralMode.Brake);
  }

  private void intake() {
    grabber.setPercentOut(0.5); // TODO: find best value
  }

  private void outake() {
    grabber.setPercentOut(-0.5); // TODO: find best value
  }

  private void stop() {
    grabber.setPercentOut(0); // might want to use PID hold
  }

  private void open() {
    solenoidTop.set(Value.kReverse);
    solenoidBottom.set(Value.kReverse);
  }

  private void close() {
    solenoidTop.set(Value.kForward);
    solenoidBottom.set(Value.kForward);
  }

  public CommandBase intakeCommand() {
    return new RunCommand(() -> {
      if (!limitSwitch.get()) {
        intake();
      } else {
        stop();
      }
    }, this);
  }

  public CommandBase releaseCommand() {
    return new InstantCommand(() -> open(), this);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
