// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.components.HighlanderFalcon;
import frc.lib.components.ReversibleDigitalInput;
import frc.lib.logging.LoggingWrapper;
import frc.robot.Constants;

public class GreybotsGrabberSubsystem extends SubsystemBase {
  static HighlanderFalcon grabberIntake = new HighlanderFalcon(Constants.MechanismConstants.grabberIntakeID, "CANivore", 1, 5e-1, 0.0, 0.0);
  static HighlanderFalcon grabberPivot = new HighlanderFalcon(
    Constants.MechanismConstants.grabberPivotID, 
    "CANivore",
    1.0, 
    2.0e-2, 
    0.0, 
    0.0);
  ReversibleDigitalInput resetLimitSwitch = new ReversibleDigitalInput(Constants.MechanismConstants.grabberLimitSwitch, true);
  ReversibleDigitalInput cubeBeambreak = new ReversibleDigitalInput(Constants.MechanismConstants.grabberBeambreak, true);
  DutyCycleEncoder absEncoder = new DutyCycleEncoder(Constants.ArmConstants.armEncoderID);
  LinearFilter intakeCurrentFilter = LinearFilter.movingAverage(50);
  LinearFilter pivotCurrentFilter = LinearFilter.movingAverage(10);

  public GamePiece gamePiece = GamePiece.None;

  public static enum GamePiece {
    None,
    Cone,
    Cube
  }

  /** Creates a new GrabberSubsystem. */
  public GreybotsGrabberSubsystem() {
    grabberIntake.configVoltageCompSaturation(10);
    grabberIntake.setNeutralMode(NeutralMode.Brake);
    grabberPivot.setNeutralMode(NeutralMode.Brake);
    grabberPivot.config_kF(0, 0);
    grabberPivot.setInverted(TalonFXInvertType.CounterClockwise);
    // grabberPivot.setSensorPhase(true);
    goToStorageRotation();
  }

  public boolean getBeambreak() {
    return cubeBeambreak.get();
  }

  public boolean getIsExtended() {
    return grabberPivot.getSelectedSensorPosition() > 1.0e4;
  }

  /** dont use. gonna just ignore the abs sensor for now, since teeth can skip */
  public void resetEncoderToAbs() {
    grabberPivot.setSelectedSensorPosition(absEncoder.get() * 2048 * 20);
  }

  public void resetEncoderToZero() {
    grabberPivot.setSelectedSensorPosition(0);
  }

  private void intakeCube() {
    grabberIntake.setPercentOut(-0.65); // TODO: find best value
  }

  public void holdCube() {
    grabberIntake.setPercentOut(-0.15);
  }

  private void outakeCube() {
    grabberIntake.setPercentOut(0.4); // TODO: find best value
  }

  public void intakeCone() {
    grabberIntake.setPercentOut(0.6);
  }

  private static void outakeCone() {
    grabberIntake.setPercentOut(-0.3);
  }

  private void goToSingleSubstationRotation() {
    grabberPivot.set(ControlMode.Position, Constants.MechanismConstants.grabberSingleSubstationRotation);
  }

  private void goToDoubleSubstationRotation() {
    grabberPivot.set(ControlMode.Position, Constants.MechanismConstants.grabberDoubleSubstationRotation);
  }

  private void goToScoringRotation() {
    grabberPivot.set(ControlMode.Position, Constants.MechanismConstants.grabberScoringRotation);
  }

  private void goToRoutingRotation() {
    grabberPivot.set(ControlMode.Position, Constants.MechanismConstants.grabberRoutingRotation);
  }

  private void goToStorageRotation() {
    grabberPivot.set(ControlMode.Position, Constants.MechanismConstants.grabberStoringRotation);
  }

  public void stop() {
    // grabber.set(ControlMode.Velocity, 0); // might want to use PID hold
    grabberIntake.setPercentOut(0);
    grabberPivot.set(ControlMode.PercentOutput, 0);
  }

  public CommandBase intakeCubeCommand(){
    return new RunCommand(() -> {intakeCube(); goToRoutingRotation();}, this)
      .until(() -> cubeBeambreak.get())
      .andThen(new WaitCommand(0.4))
      .finallyDo((boolean interrupt) -> {stop(); gamePiece = GamePiece.Cube;});
  }

  public CommandBase intakeConeDoubleCommand(){
    return new InstantCommand(() -> {intakeCurrentFilter.reset(); goToDoubleSubstationRotation();}).andThen(
      new RunCommand(() -> intakeCone(), this))
      .until(() -> intakeCurrentFilter.calculate(grabberIntake.getStatorCurrent()) > 60.0)
      .andThen(
        new RunCommand(() -> intakeCone(), this).withTimeout(0.4),
        new InstantCommand(() -> stop()), 
        new InstantCommand(() -> gamePiece = GamePiece.Cone),
        runToRoutingCommand());
  }
  
  public CommandBase intakeConeSingleCommand(){
    return new InstantCommand(() -> {intakeCurrentFilter.reset(); goToSingleSubstationRotation();}).andThen(
      new RunCommand(() -> intakeCone(), this))
      .until(() -> intakeCurrentFilter.calculate(grabberIntake.getStatorCurrent()) > 60.0)
      .andThen(
        new InstantCommand(() -> gamePiece = GamePiece.Cone),
        new RunCommand(() -> intakeCone(), this).withTimeout(0.4),
        new InstantCommand(() -> stop()), 
        runToRoutingCommand());
  }

  public CommandBase intakeConeSingleContinuousCommand() {
    return new RunCommand(() -> intakeCone()).alongWith(runToSingleSubstationCommand());
  }

  public CommandBase outakeCubeCommand(){
    return new RunCommand(() -> {outakeCube(); gamePiece = GamePiece.None;}, this);
  }

  public CommandBase outakeConeCommand(){
    return new RunCommand(() -> {outakeCone(); gamePiece = GamePiece.None;}, this);
  }

  public CommandBase runToStorageCommand() {
    return new RunCommand(() -> goToStorageRotation(), this);
  }

  public CommandBase runToRoutingCommand() {
    return new RunCommand(() -> goToRoutingRotation(), this);
  }

  public CommandBase runToScoringCommand() {
    return new RunCommand(() -> goToScoringRotation(), this);
  }

  public CommandBase runToSingleSubstationCommand() {
    return new RunCommand(() -> goToSingleSubstationRotation(), this);
  }

  public CommandBase runToDoubleSubstationCommand() {
    return new RunCommand(() -> goToDoubleSubstationRotation(), this);
  }

  public CommandBase runToRoutingStopCommand() {
    return runToRoutingCommand()
      .alongWith(new ConditionalCommand(
        new RunCommand(() -> grabberIntake.set(ControlMode.PercentOutput, 0.2))
          .withTimeout(0.5).andThen(new RunCommand(() -> grabberIntake.set(ControlMode.PercentOutput, 0))), 
        new RunCommand(() -> grabberIntake.set(ControlMode.PercentOutput, 0)),
        () -> gamePiece == GamePiece.Cone));
  }

  public CommandBase scoreConeCommand() {
    return runToScoringCommand().raceWith(new RunCommand(() -> grabberIntake.setPercentOut(0)))
      .raceWith(new WaitCommand(0.2).andThen(new WaitUntilCommand(() -> grabberIntake.getClosedLoopError() < 500.0)))
      .andThen(
        outakeConeCommand().withTimeout(0.5)
      );
  }

  public CommandBase scoreCubeCommand() {
    return runToRoutingCommand().until(() -> grabberIntake.getClosedLoopError() < 1000.0)
      .andThen(
        outakeCubeCommand().withTimeout(0.5)
      );
  }

  public Command stopCommand() {
    return new RunCommand(() -> {stop();}, this);
  }

  public CommandBase resetPivotCommand() {
    return new FunctionalCommand(
      () -> grabberIntake.set(ControlMode.PercentOutput, 0.0), 
      () -> grabberPivot.set(ControlMode.PercentOutput, -0.2), 
      (interrupt) -> {if (!interrupt) {resetEncoderToZero();} stop();}, 
      () -> resetLimitSwitch.get(),
      this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (resetLimitSwitch.get()) {
      resetEncoderToZero();
    }

    LoggingWrapper.shared.add("grabber output", grabberIntake.getMotorOutputVoltage());
    LoggingWrapper.shared.add("grabber error", grabberIntake.getClosedLoopError(0));
    LoggingWrapper.shared.add("grabber pivot angle", grabberPivot.getSelectedSensorPosition());
    LoggingWrapper.shared.add("grabber pivot target", grabberPivot.getClosedLoopTarget());
    LoggingWrapper.shared.add("grabber reset limit switch", resetLimitSwitch.get());
    LoggingWrapper.shared.add("grabber beambreak", cubeBeambreak.get());
  }
}
