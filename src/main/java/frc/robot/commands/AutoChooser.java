package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem.ScoringLevels;
import frc.robot.subsystems.Grabber.GrabberSubsystem;
import frc.robot.subsystems.Grabber.GrabberSubsystem.GamePiece;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Routing.RoutingSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoChooser {
  LoggedDashboardChooser<Supplier<Command>> chooser =
      new LoggedDashboardChooser<Supplier<Command>>("Auto Chooser");
  SwerveSubsystem swerveSubsystem;
  IntakeSubsystem intakeSubsystem;
  ElevatorSubsystem elevatorSubsystem;
  GrabberSubsystem grabberSubsystem;
  RoutingSubsystem routingSubsystem;
  HashMap<String, Command> eventMap = new HashMap<>();

  public AutoChooser(
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

    eventMap.put("Rezero Grabber", grabberSubsystem.resetPivotCommand().asProxy());
    eventMap.put(
        "Score No Aim",
        new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L3, true))
            .alongWith(new InstantCommand(() -> grabberSubsystem.gamePiece = GamePiece.Cone))
            .andThen(superstructureSubsystem.scoreNoAim().asProxy().andThen(new WaitCommand(0.5))));
    eventMap.put(
        "Extend L3",
        new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L3, true))
            .asProxy()
            .alongWith(new InstantCommand(() -> grabberSubsystem.gamePiece = GamePiece.Cube)));
    eventMap.put(
        "Score L3 No Aim",
        new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L3, true))
            .alongWith(new InstantCommand(() -> grabberSubsystem.gamePiece = GamePiece.Cone))
            .andThen(superstructureSubsystem.scoreNoAim().asProxy().andThen(new WaitCommand(0.5))));
    eventMap.put(
        "Score L3 No Aim No Wait",
        new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L3, true))
            .alongWith(new InstantCommand(() -> grabberSubsystem.gamePiece = GamePiece.Cone))
            .andThen(superstructureSubsystem.scoreNoAim().asProxy()));
    eventMap.put(
        "Score L2 No Aim",
        new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L2, true))
            .alongWith(new InstantCommand(() -> grabberSubsystem.gamePiece = GamePiece.Cone))
            .andThen(superstructureSubsystem.scoreNoAim().asProxy().andThen(new WaitCommand(0.5))));
    eventMap.put(
        "Score L2 No Aim No Wait",
        new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L2, true))
            .alongWith(new InstantCommand(() -> grabberSubsystem.gamePiece = GamePiece.Cone))
            .andThen(superstructureSubsystem.scoreNoAim().asProxy()));
    eventMap.put(
        "Score L2 No Aim Cube",
        new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L2, false))
            .alongWith(new InstantCommand(() -> grabberSubsystem.gamePiece = GamePiece.Cube))
            .andThen(superstructureSubsystem.scoreNoAim().asProxy().andThen(new WaitCommand(0.5))));
    eventMap.put(
        "Score L2 No Aim Cube No Wait",
        new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L2, false))
            .alongWith(new InstantCommand(() -> grabberSubsystem.gamePiece = GamePiece.Cube))
            .andThen(superstructureSubsystem.scoreNoAim().asProxy()));
    eventMap.put(
        "Score L2 No Aim Cube Almost No Wait",
        new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L2, false))
            .alongWith(new InstantCommand(() -> grabberSubsystem.gamePiece = GamePiece.Cube))
            .andThen(
                superstructureSubsystem.scoreNoAim().asProxy().andThen(new WaitCommand(0.25))));
    eventMap.put(
        "Score L3 No Aim Cube",
        new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L3, false))
            .alongWith(new InstantCommand(() -> grabberSubsystem.gamePiece = GamePiece.Cube))
            .andThen(superstructureSubsystem.scoreNoAim().asProxy().andThen(new WaitCommand(0.5))));
    eventMap.put(
        "Score L3 No Aim Cube No Wait",
        new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L3, false))
            .alongWith(new InstantCommand(() -> grabberSubsystem.gamePiece = GamePiece.Cube))
            .andThen(superstructureSubsystem.scoreNoAim().asProxy()));
    eventMap.put("Test Wait", new WaitCommand(1.0));
    eventMap.put("Balance", swerveSubsystem.autoBalance());
    eventMap.put("Outake Cube", grabberSubsystem.outakeCubeCommand().asProxy());
    eventMap.put(
        "Outake Cube Ground",
        run(
                intakeSubsystem.outakeCommand(),
                routingSubsystem.outakeCommand(),
                grabberSubsystem.outakeCubeCommand())
            .withTimeout(1.0)
            .asProxy());
    eventMap.put(
        "Intake",
        run(
                intakeSubsystem.runCommand().withTimeout(1.0),
                routingSubsystem.runCommand(),
                grabberSubsystem.intakeCubeCommand())
            .withTimeout(1.3)
            .asProxy());
    eventMap.put(
        "Run Up Charge Station",
        swerveSubsystem
            .driveCommand(() -> 1.0, () -> 0.0, () -> 0.0, false, false, false)
            .withTimeout(0.5)
            .finallyDo(
                (boolean interrupt) ->
                    swerveSubsystem.drive(new Translation2d(), 0, false, false, false)));
    eventMap.put("Zero Elevator", elevatorSubsystem.zeroElevator());

    chooser.addDefaultOption("none", () -> new InstantCommand(() -> {}));
    // "Two Cone Auto",
    // new TwoConeAuto(swerveSubsystem, intakeSubsystem));
    chooser.addOption("2 + Charge Flat", () -> twoParkTop());
    chooser.addOption("1.5 + Charge Flat", () -> onePlusParkTop());
    chooser.addOption("1 + Charge Middle", () -> onePlusParkMiddle());
    chooser.addOption("1.5 Bump", () -> oneAndAHalfBottom());
    chooser.addOption("1.5 + Park Bump", () -> onePlusParkBottom());
    chooser.addOption("1 + Mobility", () -> oneMobility());
    chooser.addOption("Apriltags Test", () -> apriltagsTest());
    // chooser.addOption("2 + Park Bottom Red", twoPlusParkBottomRed());
    chooser.addOption("2 Bump", () -> twoPieceBump());
    chooser.addOption("2.5 Piece Top", () -> twoAndAHalfTop());
    chooser.addOption("3 Piece Flat", () -> threeTop());
    chooser.addOption("2 Piece Middle", () -> twoPieceMiddle());
    chooser.addOption("3 Piece Bottom", () -> threeBottom());
    chooser.addOption("Better 2 Bottom", () -> twoPieceBumpBetter());
    chooser.addOption(
        "just score",
        () ->
            new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L3, true))
                .alongWith(new InstantCommand(() -> grabberSubsystem.gamePiece = GamePiece.Cone))
                .andThen(superstructureSubsystem.scoreNoAim().asProxy()));

    // LoggingWrapper.shared.add("autoChooser", chooser);

    // List<PathPlannerTrajectory> twoConeGroup = PathPlanner.loadPathGroup
    // ("TwoCone", new PathConstraints(4, 3));
    // HashMap<String, Command> Constants.eventMap = new HashMap<>();
  }

  public Command getAutoCommand() {
    return chooser.get().get();
  }

  private Command apriltagsTest() {
    return swerveSubsystem
        .autoBuilder(eventMap)
        .fullAuto(chooseAutoAlliance("Apriltags Test", "Apriltags Test"));
  }

  private Command onePlusParkTop() {
    return auto("1 + Park Top");
  }

  private Command twoParkTop() {
    return auto("2 + Park Top");
  }

  private Command onePlusParkMiddle() {
    return auto("Middle 1 + Balance", 1.5, 2.0);
  }

  private Command onePlusParkBottom() {
    return auto("1.5 + Park Bottom");
  }

  public Command oneAndAHalfBottom() {
    return auto("1.5 Bottom");
  }

  public Command oneMobility() {
    return auto("1 + Mobility Blue");
  }

  public Command threeTop() {
    return auto("3 Piece Top", 3.5, 5.0);
  }

  public Command threeBottom() {
    return auto("3 Piece Bottom");
  }

  public Command twoPieceBump() {
    return auto("2 Bottom", 2.0, 2.0);
  }

  public Command twoPieceMiddle() {
    return auto("2 + Park Middle");
  }

  public Command twoPieceBumpBetter() {
    return auto("Better 2 Bottom");
  }

  public Command twoAndAHalfTop() {
    return auto("2.5 Piece Top");
  }

  private static Command run(Command... commands) {
    return new ParallelCommandGroup(commands);
  }

  private List<PathPlannerTrajectory> chooseAutoAlliance(String nameBlue, String nameRed) {
    return chooseAutoAlliance(Constants.AutoConstants.autoConstraints, nameBlue, nameRed);
  }

  private List<PathPlannerTrajectory> chooseAutoAlliance(
      double maxVel, double maxAcc, String nameBlue, String nameRed) {
    return chooseAutoAlliance(new PathConstraints(maxVel, maxAcc), nameBlue, nameRed);
  }

  private List<PathPlannerTrajectory> chooseAutoAlliance(
      PathConstraints constraints, String nameBlue, String nameRed) {
    if (DriverStation.getAlliance() == Alliance.Blue) {
      List<PathPlannerTrajectory> pathBlue = PathPlanner.loadPathGroup(nameBlue, constraints);
      return pathBlue;
    } else {
      List<PathPlannerTrajectory> pathRed = PathPlanner.loadPathGroup(nameRed, constraints);
      return pathRed;
    }
  }

  private CommandBase auto(String name) {
    return swerveSubsystem
        .autoBuilder(eventMap)
        .fullAuto(chooseAutoAlliance(name + " Blue", name + " Red"));
  }

  private CommandBase auto(String name, double maxVel, double maxAcc) {
    return swerveSubsystem
        .autoBuilder(eventMap)
        .fullAuto(chooseAutoAlliance(maxVel, maxAcc, name + " Blue", name + " Red"));
  }
}
