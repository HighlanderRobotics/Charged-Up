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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
          .andThen(superstructureSubsystem.scoreNoAim().asProxy()));
        eventMap.put("Score L3 No Aim", 
          new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L3, true))
          .alongWith(new InstantCommand(() -> greybotsGrabberSubsystem.gamePiece = GamePiece.Cone))
          .andThen(superstructureSubsystem.scoreNoAim().asProxy()));
        eventMap.put("Score L2 No Aim", 
          new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L2, true))
          .alongWith(new InstantCommand(() -> greybotsGrabberSubsystem.gamePiece = GamePiece.Cone))
          .andThen(superstructureSubsystem.scoreNoAim().asProxy()));
        eventMap.put("Score L2 No Aim Cube", 
          new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L2, true))
          .alongWith(new InstantCommand(() -> greybotsGrabberSubsystem.gamePiece = GamePiece.Cube))
          .andThen(superstructureSubsystem.scoreNoAim().asProxy()));
        eventMap.put("Score L3 No Aim Cube", 
          new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L3, false))
          .alongWith(new InstantCommand(() -> greybotsGrabberSubsystem.gamePiece = GamePiece.Cube))
          .andThen(superstructureSubsystem.scoreNoAim().asProxy()));
        eventMap.put("Test Wait", new WaitCommand(2.0));
        eventMap.put("Balance", swerveSubsystem.autoBalance());
        eventMap.put("Intake", run(
          intakeSubsystem.runCommand(), 
          routingSubsystem.runCommand(), 
          greybotsGrabberSubsystem.intakeCubeCommand())
          .withTimeout(4.0).asProxy()); 
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
        // chooser.addOption("2 + Park Bottom Red", twoPlusParkBottomRed());
        chooser.addOption("2 Bump", () -> twoPieceBump());
        chooser.addOption("3 Piece Flat", () -> threeTop());
        chooser.addOption("2 Piece Middle", () -> twoPieceMiddle());
        chooser.addOption("just score", () ->  
        new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L2, true))
        .alongWith(new InstantCommand(() -> greybotsGrabberSubsystem.gamePiece = GamePiece.Cone))
        .andThen(superstructureSubsystem.scoreNoAim().asProxy()));

        SmartDashboard.putData("autoChooser", chooser);

        //List<PathPlannerTrajectory> twoConeGroup = PathPlanner.loadPathGroup
    //("TwoCone", new PathConstraints(4, 3));
    //HashMap<String, Command> Constants.eventMap = new HashMap<>();  
    }
    
  public Command getAutoCommand(){
    return chooser.getSelected().get();
  }
  private Command apriltagsTest() {
    return swerveSubsystem.autoBuilder(eventMap).fullAuto(chooseAutoAlliance("Apriltags Test", "Apriltags Test"));
  }
  private Command onePlusParkTop(){
    return swerveSubsystem.autoBuilder(eventMap).fullAuto(chooseAutoAlliance("1 + Park Top Blue", "1 + Park Top Red"));
  }
  private Command twoParkTop(){
    return swerveSubsystem.autoBuilder(eventMap).fullAuto(chooseAutoAlliance("2 + Park Top Blue", "2 + Park Top Red"));
  }

  private Command onePlusParkMiddle() {
    return swerveSubsystem.autoBuilder(eventMap).fullAuto(chooseAutoAlliance("Middle 1 + Balance Blue", "Middle 1 + Balance Red"));
  }

  private Command onePlusParkBottom() {
    return swerveSubsystem.autoBuilder(eventMap).fullAuto(chooseAutoAlliance("1.5 + Park Bottom Blue", "1.5 + Park Bottom Red"));
  }

  public Command oneAndAHalfBottom() {
    return swerveSubsystem.autoBuilder(eventMap).fullAuto(chooseAutoAlliance("1.5 Bottom Blue", "1.5 Bottom Red"));
  }

  public Command oneMobility() {
    return swerveSubsystem.autoBuilder(eventMap).fullAuto(chooseAutoAlliance("1 + Mobility Blue", "1 + Mobility Red"));
  }

  public Command threeTop() {
    return swerveSubsystem.autoBuilder(eventMap).fullAuto(chooseAutoAlliance("3 Piece Top Blue", "3 Piece Top Red"));
  }
  public Command twoPlusParkBottomRed(){
    return swerveSubsystem.autoBuilder(eventMap).fullAuto(chooseAutoAlliance("3 Piece Top Blue", "2 + Park Bottom Red"));
  }

  public Command twoPieceBump() {
    return swerveSubsystem.autoBuilder(eventMap).fullAuto(chooseAutoAlliance("2 Bottom Blue", "2 Bottom Red"));
  }

  public Command twoPieceMiddle() {
    return swerveSubsystem.autoBuilder(eventMap).fullAuto(chooseAutoAlliance("2 + Park Middle Blue", "2 + Park Middle Red"));
  }

  private static Command run(Command ... commands) {
    return new ParallelCommandGroup(commands);
  } 

  private List<PathPlannerTrajectory> chooseAutoAlliance(String nameBlue, String nameRed) {
    if (DriverStation.getAlliance() == Alliance.Blue) {
      List<PathPlannerTrajectory> pathBlue = PathPlanner.loadPathGroup(nameBlue, new PathConstraints(Constants.AutoConstants.maxSpeedMetersPerSecond, Constants.AutoConstants.maxAccelerationMetersPerSecondSquared));
      return pathBlue;
    } else {
      List<PathPlannerTrajectory> pathRed = PathPlanner.loadPathGroup(nameRed, new PathConstraints(Constants.AutoConstants.maxSpeedMetersPerSecond, Constants.AutoConstants.maxAccelerationMetersPerSecondSquared));
      return pathRed;
    }
  }
}
