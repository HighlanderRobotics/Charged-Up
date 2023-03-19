// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.components.HighlanderFalcon;

import frc.robot.Constants;

public class GreybotsGrabberSubsystem extends SubsystemBase {
  HighlanderFalcon grabber = new HighlanderFalcon(Constants.MechanismConstants.grabberID, "CANivore", 1, 5e-1, 0.0, 0.0);


  
  /** Creates a new GrabberSubsystem. */
  public GreybotsGrabberSubsystem() {
    grabber.configVoltageCompSaturation(10);
  }

  private void intakeCube() {
    grabber.setPercentOut(-0.9); // TODO: find best value
  }

  private void outakeCube() {
    grabber.setPercentOut(0.9); // TODO: find best value
  }

  private void intakeCone(){
    grabber.setPercentOut(0.9);
  }

  private void outakeCone(){
    grabber.setPercentOut(-0.9);
  }


  private void stop() {
    // grabber.set(ControlMode.Velocity, 0); // might want to use PID hold
    grabber.setPercentOut(0);
  }
  
  public CommandBase intakeCubeCommand(){
    return new RunCommand(() -> {intakeCube();}, this);
  }

  public CommandBase intakeConeCommand(){
    return new RunCommand(() -> {intakeCone();}, this);
  }

  public CommandBase outakeCubeCommand(){
    return new RunCommand(() -> {outakeCube();}, this);
  }

  public CommandBase outakeConeCommand(){
    return new RunCommand(() -> {outakeCone();}, this);
  }

  @Override
 public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("grabber output", grabber.getMotorOutputPercent());

  }
}
