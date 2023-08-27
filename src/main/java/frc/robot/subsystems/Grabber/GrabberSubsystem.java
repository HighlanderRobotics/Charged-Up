// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Grabber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Grabber.GrabberIO.GrabberIOInputs;

public class GrabberSubsystem extends SubsystemBase {
  GrabberIOFalcon io = new GrabberIOFalcon();
  GrabberIOInputsAutoLogged inputs = new GrabberIOInputsAutoLogged();

  LinearFilter intakeCurrentFilter = LinearFilter.movingAverage(50);
  LinearFilter pivotCurrentFilter = LinearFilter.movingAverage(10);

  public GamePiece gamePiece = GamePiece.None;

  public static enum GamePiece {
    None,
    Cone,
    Cube
  }

  /** Creates a new GrabberSubsystem. */
  public GrabberSubsystem() {

    // grabberPivot.setSensorPhase(true);
    goToStorageRotation();
  }

  public boolean getBeambreak() {
    return io.getBeambreak();
  }

  public boolean getIsExtended() {
    return io.getPivotPosition() > 1.0e4;
  }

  public void resetEncoderToZero() {
    io.resetPivotEncoder();
  }

  private void intakeCube() {
    io.setRollersPercentOut(-0.65); // TODO: find best value
  }

  public void holdCube() {
    io.setRollersPercentOut(-0.15);
  }

  private void outakeCube() {
    io.setRollersPercentOut(0.4); // TODO: find best value
  }

  public void intakeCone() {
    io.setRollersPercentOut(0.8);
  }

  private void outakeCone() {
    io.setRollersPercentOut(-0.3);
  }

  private void goToSingleSubstationRotation() {
    io.setPivotTarget(Constants.MechanismConstants.grabberSingleSubstationRotation);
  }

  private void goToDoubleSubstationRotation() {
    io.setPivotTarget(Constants.MechanismConstants.grabberDoubleSubstationRotation);
  }

  private void goToScoringRotation() {
    io.setPivotTarget(Constants.MechanismConstants.grabberScoringRotation);
  }

  private void goToRoutingRotation() {
    io.setPivotTarget(Constants.MechanismConstants.grabberRoutingRotation);
  }

  private void goToStorageRotation() {
    io.setPivotTarget(Constants.MechanismConstants.grabberStoringRotation);
  }

  public void stop() {
    // grabber.set(ControlMode.Velocity, 0); // might want to use PID hold
    io.setRollersPercentOut(0);
    io.setPivotPercentOut(0);
  }

  public CommandBase intakeCubeCommand() {
    return new RunCommand(
            () -> {
              intakeCube();
              goToRoutingRotation();
            },
            this)
        .until(() -> io.getBeambreak())
        .andThen(new WaitCommand(0.4))
        .finallyDo(
            (boolean interrupt) -> {
              stop();
              gamePiece = GamePiece.Cube;
            });
  }

  public CommandBase intakeConeDoubleCommand() {
    return new InstantCommand(
            () -> {
              intakeCurrentFilter.reset();
              goToDoubleSubstationRotation();
            })
        .andThen(new RunCommand(() -> intakeCone(), this))
        .until(() -> intakeCurrentFilter.calculate(inputs.rollersCurrentAmps) > 60.0)
        .andThen(
            new RunCommand(() -> intakeCone(), this).withTimeout(0.4),
            new InstantCommand(() -> stop()),
            new InstantCommand(() -> gamePiece = GamePiece.Cone),
            runToRoutingCommand());
  }

  public CommandBase intakeConeSingleCommand() {
    return new InstantCommand(
            () -> {
              intakeCurrentFilter.reset();
              goToSingleSubstationRotation();
            })
        .andThen(new RunCommand(() -> intakeCone(), this))
        .until(() -> intakeCurrentFilter.calculate(inputs.rollersCurrentAmps) > 60.0)
        .andThen(
            new InstantCommand(() -> gamePiece = GamePiece.Cone),
            new RunCommand(() -> intakeCone(), this).withTimeout(0.4),
            new InstantCommand(() -> stop()),
            runToRoutingCommand());
  }

  public CommandBase intakeConeSingleContinuousCommand() {
    return new RunCommand(() -> intakeCone()).alongWith(runToSingleSubstationCommand());
  }

  public CommandBase outakeCubeCommand() {
    return new RunCommand(
        () -> {
          outakeCube();
          gamePiece = GamePiece.None;
        },
        this);
  }

  public CommandBase outakeConeCommand() {
    return new RunCommand(
        () -> {
          outakeCone();
          gamePiece = GamePiece.None;
        },
        this);
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
        .alongWith(
            new ConditionalCommand(
                new RunCommand(() -> io.setRollersPercentOut(0.2))
                    .withTimeout(0.5)
                    .andThen(new RunCommand(() -> io.setRollersPercentOut(0))),
                new RunCommand(() -> io.setRollersPercentOut(0)),
                () -> gamePiece == GamePiece.Cone));
  }

  public CommandBase scoreConeCommand() {
    return runToScoringCommand()
        .raceWith(new RunCommand(() -> io.setRollersPercentOut(0)))
        .raceWith(
            new WaitCommand(0.2) // wait why did this work? the intake is open loop . . .
                .andThen(new WaitUntilCommand(() -> io.getRollersError() < 500.0)))
        .andThen(outakeConeCommand().withTimeout(0.5));
  }

  public CommandBase scoreCubeCommand() {
    return runToRoutingCommand()
        .until(() -> io.getRollersError() < 1000.0)
        .andThen(outakeCubeCommand().withTimeout(0.5));
  }

  public Command stopCommand() {
    return new RunCommand(
        () -> {
          stop();
        },
        this);
  }

  public CommandBase resetPivotCommand() {
    
    return new FunctionalCommand(
        () -> io.setRollersPercentOut(0.0),
        () -> io.setPivotPercentOut(-0.2),
        (interrupt) -> {
          if (!interrupt) {
            resetEncoderToZero();
          }
          stop();
        },
        () -> io.getLimitSwitch(),
        this);


  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Grabber", inputs);

    if (io.getLimitSwitch()) {
      resetEncoderToZero();
      Logger.getInstance().recordOutput("grabber reset to 0", true);
    }
    
    // LoggingWrapper.shared.add("grabber output", grabberIntake.getMotorOutputVoltage());
    // LoggingWrapper.shared.add("grabber error", grabberIntake.getClosedLoopError(0));
    // LoggingWrapper.shared.add("grabber pivot angle", grabberPivot.getSelectedSensorPosition());
    // LoggingWrapper.shared.add("grabber pivot angle 2", grabberPivot.getSelectedSensorPosition());
    // LoggingWrapper.shared.add("grabber pivot target", grabberPivot.getClosedLoopTarget());
    // LoggingWrapper.shared.add("grabber reset limit switch", resetLimitSwitch.get());
    // LoggingWrapper.shared.add("grabber beambreak", cubeBeambreak.get());
  }
}
