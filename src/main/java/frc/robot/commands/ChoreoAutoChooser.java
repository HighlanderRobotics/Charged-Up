package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.choreolib.AutoFieldPosition;
import frc.lib.choreolib.ChoreoPath;
import frc.lib.choreolib.ChoreoPathFeature;
import frc.lib.choreolib.ChoreoTrajectory;
import frc.lib.choreolib.TrajectoryManager;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem.ScoringLevels;
import frc.robot.subsystems.Grabber.GrabberSubsystem;
import frc.robot.subsystems.Grabber.GrabberSubsystem.GamePiece;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Routing.RoutingSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public final class ChoreoAutoChooser {
  LoggedDashboardChooser<Command> chooser =
      new LoggedDashboardChooser<Command>("Choreo Auto Chooser");
  SwerveSubsystem swerveSubsystem;
  IntakeSubsystem intakeSubsystem;
  ElevatorSubsystem elevatorSubsystem;
  GrabberSubsystem grabberSubsystem;
  RoutingSubsystem routingSubsystem;
  SuperstructureSubsystem superstructureSubsystem;

  public ChoreoAutoChooser(
      SwerveSubsystem swerveSubsystem,
      IntakeSubsystem intakeSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      RoutingSubsystem routingSubsystem,
      GrabberSubsystem grabberSubsystem,
      SuperstructureSubsystem superstructureSubsystem) {

    this.intakeSubsystem = intakeSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.grabberSubsystem = grabberSubsystem;
    this.routingSubsystem = routingSubsystem;
    this.superstructureSubsystem = superstructureSubsystem;

    TrajectoryManager.getInstance().LoadTrajectories();

    for (ChoreoPath path : paths) {
      chooser.addOption(path.localizedDescription(), runAuto(path));
    }

    chooser.addDefaultOption("Just Score", justScore());
  }

  public Command getAutonomousCommand() {
    return chooser.get();
  }

  public final ChoreoPath[] paths =
      new ChoreoPath[] {
        new ChoreoPath(2, AutoFieldPosition.Bump, ChoreoPathFeature.none),
        new ChoreoPath(2, AutoFieldPosition.Clear, ChoreoPathFeature.none),
        new ChoreoPath(3, AutoFieldPosition.Clear, ChoreoPathFeature.none),
        new ChoreoPath(3, AutoFieldPosition.Bump, ChoreoPathFeature.none)
      };

  public ChoreoTrajectory getPath(ChoreoPath path) {
    return TrajectoryManager.getInstance()
        .getTrajectory("individual_trajectories/" + path.fileName());
  }

  /** Extend and intake for 1.3 sec */
  private Command intake() {
    return Commands.parallel(
            intakeSubsystem.runCommand().withTimeout(1.0),
            routingSubsystem.runCommand(),
            grabberSubsystem.intakeCubeCommand())
        .withTimeout(1.3)
        .asProxy();
  }

  /** Extend and outake for 1.0 sec */
  private Command outake() {
    return Commands.parallel(
            intakeSubsystem.outakeCommand(),
            routingSubsystem.outakeCommand(),
            grabberSubsystem.outakeCubeCommand())
        .withTimeout(1.0)
        .asProxy();
  }

  private Command runAuto(ChoreoPath path) {
    System.out.println("getting trajectory: " + path.fileName());
    if (path.pieceCount == 2) {
      return twoPiece(path);
    } else if (path.pieceCount == 3) {
      if (path.fieldPosition == AutoFieldPosition.Clear) {
        return threePieceClear(path);
      } else {
        return threePieceBump(path);
      }
    } else {
      return justScore();
    }
  }

  private Command scoreLevelThree() {
    return new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L3, true))
        .alongWith(new InstantCommand(() -> grabberSubsystem.gamePiece = GamePiece.Cone))
        .andThen(superstructureSubsystem.scoreNoAim().asProxy())
        .andThen(new WaitCommand(0.8));
  }

  private Command twoPiece(ChoreoPath path) {
    return Commands.sequence(
        new WaitCommand(0.1),
        scoreLevelThree(),
        swerveSubsystem
            .choreoTrajFollow(getPath(path))
            .alongWith(new WaitCommand(1.6).andThen(intake())),
        scoreLevelThree());
  }

  private Command threePieceClear(ChoreoPath path) {
    return Commands.sequence(
        new WaitCommand(0.1),
        intake().withTimeout(0.5),
        swerveSubsystem
            .choreoTrajFollow(getPath(path))
            .alongWith(
                new WaitCommand(2.5).andThen(intake()),
                new WaitCommand(5.8).andThen(outake()),
                new WaitCommand(8.4).andThen(intake())),
        outake());
  }

  private Command threePieceBump(ChoreoPath path) {
    return Commands.sequence(
        new WaitCommand(0.1),
        intake().withTimeout(0.5),
        swerveSubsystem
            .choreoTrajFollow(getPath(path))
            .alongWith(
                new WaitCommand(2.2).andThen(intake()),
                new WaitCommand(6.1).andThen(outake()),
                new WaitCommand(8.2).andThen(intake())),
        outake());
  }

  private Command justScore() {
    return scoreLevelThree();
  }
}
