// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.components.HighlanderFalcon;
import frc.lib.components.ReversibleDigitalInput;
import frc.robot.Constants;

public class GreybotsGrabberSubsystem extends SubsystemBase {
  static HighlanderFalcon grabberIntake = new HighlanderFalcon(Constants.MechanismConstants.grabberIntakeID, "CANivore", 1, 5e-1, 0.0, 0.0);
  static HighlanderFalcon grabberPivot = new HighlanderFalcon(
    Constants.MechanismConstants.grabberPivotID, 
    1.0, 
    1.0e-2, 
    0.0, 
    0.0);
  ReversibleDigitalInput resetLimitSwitch = new ReversibleDigitalInput(Constants.MechanismConstants.grabberLimitSwitch, false);
  ReversibleDigitalInput cubeBeambreak = new ReversibleDigitalInput(Constants.MechanismConstants.grabberBeambreak, false);
  DutyCycleEncoder absEncoder = new DutyCycleEncoder(Constants.ArmConstants.armEncoderID);
  LinearFilter currentFilter = LinearFilter.singlePoleIIR(0.1, 0.020);

  /** Creates a new GrabberSubsystem. */
  public GreybotsGrabberSubsystem() {
    grabberIntake.configVoltageCompSaturation(10);
    grabberIntake.setNeutralMode(NeutralMode.Brake);
    grabberPivot.setNeutralMode(NeutralMode.Brake);
    grabberPivot.setInverted(TalonFXInvertType.CounterClockwise);
    grabberPivot.setSensorPhase(true);
  }

  /** dont use. gonna just ignore the abs sensor for now, since teeth can skip */
  public void resetEncoderToAbs() {
    grabberPivot.setSelectedSensorPosition(absEncoder.get() * 2048 * 20);
  }

  public void resetEncoderToZero() {
    grabberPivot.setSelectedSensorPosition(0);
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

  private static void outakeCone() {
    grabberIntake.setPercentOut(-0.9);
  }

  private void goToSingleSubstationRotation() {
    // grabberPivot.set(ControlMode.Position, Constants.MechanismConstants.grabberSingleSubstationRotation);
  }

  private void goToDoubleSubstationRotation() {
    // grabberPivot.set(ControlMode.Position, Constants.MechanismConstants.grabberDoubleSubstationRotation);
  }

  private void goToScoringRotation() {
    // grabberPivot.set(ControlMode.Position, Constants.MechanismConstants.grabberScoringRotation);
  }

  private void goToRoutingRotation() {
    // grabberPivot.set(ControlMode.Position, Constants.MechanismConstants.grabberRoutingRotation);
  }

  private void goToStorageRotation() {
    // grabberPivot.set(ControlMode.Position, Constants.MechanismConstants.grabberStoringRotation);
  }

  public void stop() {
    // grabber.set(ControlMode.Velocity, 0); // might want to use PID hold
    grabberIntake.setPercentOut(0);
    grabberPivot.setPercentOut(0);
  }

  public CommandBase intakeCubeCommand(){
    return new RunCommand(() -> {intakeCube(); goToRoutingRotation();}, this)
      .until(() -> cubeBeambreak.get())
      .finallyDo((boolean interrupt) -> stop());
  }

  public CommandBase intakeConeDoubleCommand(){
    return new InstantCommand(() -> currentFilter.reset()).andThen(
      new RunCommand(() -> {intakeCone(); goToDoubleSubstationRotation();}, this))
      .until(() -> currentFilter.calculate(grabberIntake.getStatorCurrent()) > 20.0)
      .finallyDo((boolean interrupt) -> stop());
  }
  
  public CommandBase intakeConeSingleCommand(){
    return new InstantCommand(() -> currentFilter.reset()).andThen(
      new RunCommand(() -> {intakeCone(); goToSingleSubstationRotation();}, this))
      .until(() -> currentFilter.calculate(grabberIntake.getStatorCurrent()) > 20.0)
      .finallyDo((boolean interrupt) -> stop());
  }

  public CommandBase outakeCubeCommand(){
    return new RunCommand(() -> {outakeCube();}, this);
  }

  public CommandBase outakeConeCommand(){
    return new RunCommand(() -> {outakeCone();});
  }

  public CommandBase runToStorageCommand() {
    return new InstantCommand(() -> goToStorageRotation()).andThen(new RunCommand(() -> {}));
  }

  public CommandBase runToRoutingCommand() {
    return new InstantCommand(() -> goToRoutingRotation()).andThen(new RunCommand(() -> {}));
  }

  public CommandBase runToScoringCommand() {
    return new InstantCommand(() -> goToScoringRotation()).andThen(new RunCommand(() -> {}));
  }

  public CommandBase runToSingleSubstationCommand() {
    return new InstantCommand(() -> goToSingleSubstationRotation()).andThen(new RunCommand(() -> {}));
  }

  public CommandBase runToDoubleSubstationCommand() {
    return new InstantCommand(() -> goToDoubleSubstationRotation()).andThen(new RunCommand(() -> {}));
  }

  public CommandBase scoreConeCommand() {
    return runToScoringCommand().until(() -> grabberIntake.getClosedLoopError() < 1000.0)
      .andThen(
        outakeConeCommand()
      );
  }

  public CommandBase scoreCubeCommand() {
    return runToRoutingCommand().until(() -> grabberIntake.getClosedLoopError() < 1000.0)
      .andThen(
        outakeCubeCommand()
      );
  }

  public Command stopCommand() {
    return new RunCommand(() -> {stop();}, this);
  }

  public CommandBase resetPivotCommand() {
    return new RunCommand(() -> grabberPivot.set(ControlMode.PercentOutput, -0.1), this)
      .until(() -> resetLimitSwitch.get())
      .finallyDo((boolean interrupt) -> resetEncoderToZero());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("grabber output", grabberIntake.getMotorOutputPercent());
    SmartDashboard.putNumber("grabber pivot angle", grabberPivot.getSelectedSensorPosition());
    SmartDashboard.putBoolean("grabber reset limit switch", resetLimitSwitch.get());
  }
}
