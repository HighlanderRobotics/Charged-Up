package frc.robot.commands;

import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.logging.LoggingWrapper;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GreybotsGrabberSubsystem;
import frc.robot.subsystems.RollerClawGrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RoutingSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ScoringLevels;
import frc.robot.subsystems.GreybotsGrabberSubsystem.GamePiece;

public class AutoChooser {
    SendableChooser<Supplier<Command>> chooser = new SendableChooser<Supplier<Command>>();
    SwerveSubsystem swerveSubsystem;
    IntakeSubsystem intakeSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    GreybotsGrabberSubsystem greybotsGrabberSubsystem;
    RoutingSubsystem routingSubsystem;
    HashMap<String, Command> eventMap = new HashMap<>();
    
    
    public AutoChooser(
      SwerveSubsystem swerveSubsystem,
      IntakeSubsystem intakeSubsystem, 
      ElevatorSubsystem elevatorSubsystem, 
      RoutingSubsystem routingSubsystem,
      GreybotsGrabberSubsystem greybotsGrabberSubsystem,
      SuperstructureSubsystem superstructureSubsystem){
      
        this.intakeSubsystem = intakeSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.elevatorSubsystem = elevatorSubsystem; 
        this.greybotsGrabberSubsystem = greybotsGrabberSubsystem;
        this.routingSubsystem = routingSubsystem;

        eventMap.put("Rezero Grabber", greybotsGrabberSubsystem.resetPivotCommand().asProxy());
        eventMap.put("Score", new ScoringCommand(
          ScoringLevels.L3, 
          () -> 0, 
          elevatorSubsystem, 
          swerveSubsystem, 
          greybotsGrabberSubsystem, 
          superstructureSubsystem).asProxy().andThen(
            new InstantCommand(() -> elevatorSubsystem.getDefaultCommand().schedule()),
            new PrintCommand("bbbbbbbb"), 
            new InstantCommand(() -> {}, swerveSubsystem),
            new WaitCommand(1.0),
            new PrintCommand("ccccccccccccccccccccccc")));
        eventMap.put("Score L3", new ScoringCommand(ScoringLevels.L3, () -> 0, elevatorSubsystem, swerveSubsystem, greybotsGrabberSubsystem, superstructureSubsystem));
        eventMap.put("Score No Aim", 
          new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L3, true))
          .alongWith(new InstantCommand(() -> greybotsGrabberSubsystem.gamePiece = GamePiece.Cone))
          .andThen(superstructureSubsystem.scoreNoAim().asProxy().andThen(new WaitCommand(0.5))));
        eventMap.put("Score L3 No Aim",
          new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L3, true))
          .alongWith(new InstantCommand(() -> greybotsGrabberSubsystem.gamePiece = GamePiece.Cone))
          .andThen(superstructureSubsystem.scoreNoAim().asProxy().andThen(new WaitCommand(0.5))));
        eventMap.put("Score L2 No Aim", 
          new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L2, true))
          .alongWith(new InstantCommand(() -> greybotsGrabberSubsystem.gamePiece = GamePiece.Cone))
          .andThen(superstructureSubsystem.scoreNoAim().asProxy().andThen(new WaitCommand(0.5))));
        eventMap.put("Score L2 No Aim Cube", 
          new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L2, true))
          .alongWith(new InstantCommand(() -> greybotsGrabberSubsystem.gamePiece = GamePiece.Cube))
          .andThen(superstructureSubsystem.scoreNoAim().asProxy().andThen(new WaitCommand(0.5))));
        eventMap.put("Score L3 No Aim Cube", 
          new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L3, false))
          .alongWith(new InstantCommand(() -> greybotsGrabberSubsystem.gamePiece = GamePiece.Cube))
          .andThen(superstructureSubsystem.scoreNoAim().asProxy().andThen(new WaitCommand(0.5))));
        eventMap.put("Test Wait", new WaitCommand(1.0));
        eventMap.put("Balance", swerveSubsystem.autoBalanceVelocity());
        eventMap.put("Outake Cube", new InstantCommand(() -> greybotsGrabberSubsystem.outakeCubeCommand().asProxy()));
        eventMap.put("Intake", run(
          intakeSubsystem.runCommand().withTimeout(4.0), 
          routingSubsystem.runCommand(), 
          greybotsGrabberSubsystem.intakeCubeCommand())
          .withTimeout(5.0).asProxy()); 
        eventMap.put("Run Up Charge Station", swerveSubsystem.driveCommand(
            () -> 1.0,
            () -> 0.0,
            () -> 0.0,
            false,
            false,
            false
          ).withTimeout(0.5)
          .finallyDo((boolean interrupt) -> swerveSubsystem.drive(new Translation2d(), 0, false, false, false)));
        eventMap.put("Zero Elevator", elevatorSubsystem.zeroElevator());

        chooser.setDefaultOption("none", () -> new InstantCommand(() -> {}));
            //"Two Cone Auto", 
            //new TwoConeAuto(swerveSubsystem, intakeSubsystem));
        chooser.addOption("2 + Charge Flat", () -> twoParkTop());
        chooser.addOption("1.5 + Charge Flat", () -> onePlusParkTop());
        chooser.addOption("1 + Charge Middle", () -> onePlusParkMiddle());
        chooser.addOption("1.5 Bump", () -> oneAndAHalfBottom());
        chooser.addOption("1.5 + Park Bump", () -> onePlusParkBottom());
        chooser.addOption("1 + Mobility", () -> oneMobility());
        chooser.addOption("Apriltags Test", () -> apriltagsTest());
        chooser.addOption("2.5 Piece Top Red", () -> twoAndAHalfTopRed());
        // chooser.addOption("2 + Park Bottom Red", twoPlusParkBottomRed());
        chooser.addOption("2 Bump", () -> twoPieceBump());
        chooser.addOption("3 Piece Flat", () -> threeTop());
        chooser.addOption("2 Piece Middle", () -> twoPieceMiddle());
        chooser.addOption("just score", () ->  
        
        new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L3, true))
        .alongWith(new InstantCommand(() -> greybotsGrabberSubsystem.gamePiece = GamePiece.Cone))
        .andThen(superstructureSubsystem.scoreNoAim().asProxy()));

        LoggingWrapper.shared.add("autoChooser", chooser);

        //List<PathPlannerTrajectory> twoConeGroup = PathPlanner.loadPathGroup
    //("TwoCone", new PathConstraints(4, 3));
    //HashMap<String, Command> Constants.eventMap = new HashMap<>();  
    }
    
  public Command getAutoCommand() {
    return chooser.getSelected().get();
  }
  private Command apriltagsTest() {
    return swerveSubsystem.autoBuilder(eventMap).fullAuto(chooseAutoAlliance("Apriltags Test", "Apriltags Test"));
  }
  private Command onePlusParkTop() {
    return auto("1 + Park Top");
  }
  private Command twoParkTop() {
    return auto("2 + Park Top", 3.0, 5.0);
  }

  private Command twoAndAHalfTopRed() {
    return auto("2.5 Piece Top");
  }

  private Command onePlusParkMiddle() {
    return auto("Middle 1 + Balance");
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
    return auto("3 Piece Top", 3.0, 5.0);
  }

  public Command twoPieceBump() {
    return auto("2 Bottom");
  }

  public Command twoPieceMiddle() {
    return auto("2 + Park Middle");
  }

  private static Command run(Command ... commands) {
    return new ParallelCommandGroup(commands);
  } 

  private List<PathPlannerTrajectory> chooseAutoAlliance(String nameBlue, String nameRed) {
    return chooseAutoAlliance(Constants.AutoConstants.autoConstraints, nameBlue, nameRed);
  }

  private List<PathPlannerTrajectory> chooseAutoAlliance(double maxVel, double maxAcc, String nameBlue, String nameRed) {
    return chooseAutoAlliance(new PathConstraints(maxVel, maxAcc), nameBlue, nameRed);
  }

  private List<PathPlannerTrajectory> chooseAutoAlliance(PathConstraints constraints, String nameBlue, String nameRed) {
    if (DriverStation.getAlliance() == Alliance.Blue) {
      List<PathPlannerTrajectory> pathBlue = PathPlanner.loadPathGroup(nameBlue, constraints);
      return pathBlue;
    } else {
      List<PathPlannerTrajectory> pathRed = PathPlanner.loadPathGroup(nameRed, constraints);
      return pathRed;
    }
  }

  private CommandBase auto(String name) {
    return swerveSubsystem.autoBuilder(eventMap).fullAuto(chooseAutoAlliance(name + " Blue", name + " Red"));
  }

  private CommandBase auto(String name, double maxVel, double maxAcc) {
    return swerveSubsystem.autoBuilder(eventMap).fullAuto(chooseAutoAlliance(maxVel, maxAcc, name + " Blue", name + " Red"));
  }
}
