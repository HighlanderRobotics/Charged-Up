// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.components.HighlanderFalcon;
import frc.lib.components.ReversibleDigitalInput;
import frc.robot.Constants;

public class GreybotsGrabberSubsystem extends SubsystemBase {
  HighlanderFalcon grabberIntake = new HighlanderFalcon(Constants.MechanismConstants.grabberID, "CANivore", 1, 5e-1, 0.0, 0.0);
  HighlanderFalcon grabberRotation = new HighlanderFalcon(Constants.MechanismConstants.grabberID, 1.0, 1.0, 0.0, 0.0);
  ReversibleDigitalInput limitSwitch = new ReversibleDigitalInput(Constants.MechanismConstants.grabberLimitSwitch, false);
  DutyCycleEncoder absEncoder = new DutyCycleEncoder(Constants.ArmConstants.armEncoderID);
  LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.020);

  /** Creates a new GrabberSubsystem. */
  public GreybotsGrabberSubsystem() {
    grabberIntake.configVoltageCompSaturation(10);
    resetEncoderToAbs();
  }

  public void resetEncoderToAbs() {
    grabberRotation.setSelectedSensorPosition(absEncoder.get() * 2048 * 20);
  }

  private void intakeCube() {
    grabberIntake.setPercentOut(-0.9); // TODO: find best value
  }

  private void outakeCube() {
    grabberIntake.setPercentOut(0.9); // TODO: find best value
  }

  private void intakeCone() {
    grabberIntake.setPercentOut(0.9);
  }

  private void outakeCone() {
    grabberIntake.setPercentOut(-0.9);
  }

  private void goToSingleSubstationRotation() {
    grabberRotation.set(ControlMode.Position, Constants.MechanismConstants.grabberSingleSubstationRotation);
  }

  private void goToDoubleSubstationRotation() {
    grabberRotation.set(ControlMode.Position, Constants.MechanismConstants.grabberDoubleSubstatoinRotation);
  }

  private void goToScoringSubstationRotation() {
    grabberRotation.set(ControlMode.Position, Constants.MechanismConstants.grabberScoringRotation);
  }

  private void goToRoutingRotation() {
    grabberRotation.set(ControlMode.Position, Constants.MechanismConstants.grabberRoutingRotation);
  }

  private void goToStorageRotation() {
    grabberRotation.set(ControlMode.Position, Constants.MechanismConstants.grabberStoringRotation);
  }

  public void stop() {
    // grabber.set(ControlMode.Velocity, 0); // might want to use PID hold
    grabberIntake.setPercentOut(0);
  }
  
  public CommandBase intakeCubeCommand(){
    return new InstantCommand(() -> filter.reset()).andThen(
      new RunCommand(() -> {intakeCube(); goToRoutingRotation();}, this))
      .until(() -> filter.calculate(grabberIntake.getStatorCurrent()) > 20.0);
  }

  public CommandBase intakeConeDoubleCommand(){
    return new RunCommand(() -> {intakeCone(); goToDoubleSubstationRotation();}, this);
  }
  
  public CommandBase intakeConeSingleCommand(){
    return new RunCommand(() -> {intakeCone(); goToSingleSubstationRotation();}, this);
  }

  public CommandBase outakeCubeCommand(){
    return new RunCommand(() -> {outakeCube();}, this);
  }

  public CommandBase outakeConeCommand(){
    return new RunCommand(() -> {outakeCone();}, this);
  }

  public CommandBase runToStorageCommand() {
    return new InstantCommand(() -> goToStorageRotation()).andThen(new RunCommand(() -> {}));
  }

  
  public CommandBase runToRoutingCommand() {
    return new InstantCommand(() -> goToRoutingRotation()).andThen(new RunCommand(() -> {}));
  }

  public boolean hasGamePiece() {
	  return true; //TODO
  }

  @Override
 public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("grabber output", grabberIntake.getMotorOutputPercent());

  }
}
