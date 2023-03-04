package frc.robot.commands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RoutingSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ScoringLevels;

public class AutoChooser {
    SendableChooser<Command> chooser = new SendableChooser<Command>();
    SwerveSubsystem swerveSubsystem;
    IntakeSubsystem intakeSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    ArmSubsystem armSubsystem;
    GrabberSubsystem grabberSubsystem;
    RoutingSubsystem routingSubsystem;
    HashMap<String, Command> eventMap = new HashMap<>();
    
    
    public AutoChooser(
      SwerveSubsystem swerveSubsystem,
      IntakeSubsystem intakeSubsystem, 
      ElevatorSubsystem elevatorSubsystem, 
      ArmSubsystem armSubsystem, 
      GrabberSubsystem grabberSubsystem, 
      RoutingSubsystem routingSubsystem,
      SuperstructureSubsystem superstructureSubsystem){
      
        this.intakeSubsystem = intakeSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.elevatorSubsystem = elevatorSubsystem; 
        this.armSubsystem = armSubsystem;
        this.grabberSubsystem = grabberSubsystem;
        this.routingSubsystem = routingSubsystem;

        eventMap.put("Score", new ScoringCommand(ScoringLevels.L3, () -> 0, elevatorSubsystem, swerveSubsystem, grabberSubsystem, superstructureSubsystem));
        eventMap.put("Score L3", new ScoringCommand(ScoringLevels.L3, () -> 0, elevatorSubsystem, swerveSubsystem, grabberSubsystem, superstructureSubsystem));
        eventMap.put("Score No Aim",
          superstructureSubsystem.waitExtendToInches(Constants.ScoringLevels.topConeLevel)
          .andThen(
              new PrintCommand("extended elevator"),
              new WaitCommand(0.25),
                grabberSubsystem.outakeNeutralCommand()
                .withTimeout(1.0)
            ).asProxy());
        eventMap.put("Balance", swerveSubsystem.autoBalance());
        eventMap.put("Intake", run(
          intakeSubsystem.runCommand(), 
          routingSubsystem.runCommand(), 
          grabberSubsystem.intakeOpenCommand(),
          armSubsystem.runToRoutingCommand())); 
        eventMap.put("Run Up Charge Station", swerveSubsystem.driveCommand(
            () -> 1.0,
            () -> 0.0,
            () -> 0.0,
            false,
            false,
            false
          ).withTimeout(1.0)
          .finallyDo((boolean interrupt) -> swerveSubsystem.drive(new Translation2d(), 0, false, false, false)));

        chooser.setDefaultOption("none", new InstantCommand(() -> {}));
            //"Two Cone Auto", 
            //new TwoConeAuto(swerveSubsystem, intakeSubsystem));
        chooser.addOption("2 + Park Middle Blue", parkMiddleBlue());
        chooser.addOption("NONE", new PrintCommand("owo"));
        chooser.addOption("1 + Park Bottom Blue", new PrintCommand("working"));
        chooser.addOption("2 + Park Top Blue", new PrintCommand("yes"));
        chooser.addOption("1 + Park Top Blue", onePlusParkTopBlue());
        chooser.addOption("3 Piece Top Blue", new PrintCommand("Working again"));
        chooser.addOption("3 Piece Bottom Blue", new PrintCommand("Working again"));
        chooser.addOption("GoesFiveFeet", goesFiveFeet());

        SmartDashboard.putData("autoChooser", chooser);

        //List<PathPlannerTrajectory> twoConeGroup = PathPlanner.loadPathGroup
    //("TwoCone", new PathConstraints(4, 3));
    //HashMap<String, Command> Constants.eventMap = new HashMap<>();  
    }
    
    public Command getAutoCommand(){
      return chooser.getSelected();
    }

      //change to put in constructor
      private Command parkMiddleBlue(){
        List<PathPlannerTrajectory> parkMiddleBlueGroup = PathPlanner.loadPathGroup
        ("2 + Park Middle Blue", new PathConstraints(4, 3));
        
        return new PrintCommand(eventMap.toString()).andThen(swerveSubsystem.autoBuilder(eventMap).fullAuto(parkMiddleBlueGroup));
      }
      //change to put in constuctor
      private Command twoConeAuto(){
        List<PathPlannerTrajectory> pathGroup = 
            PathPlanner.loadPathGroup(
                "twoConeAuto", new PathConstraints(4, 3));
        return swerveSubsystem.autoBuilder(eventMap).fullAuto(pathGroup);
      }

      private Command threePieceBottomBlue(){
        List<PathPlannerTrajectory> pieceBottomGroup = PathPlanner.loadPathGroup(
          "3 Piece Bottom Blue", new PathConstraints(4, 3));
        return swerveSubsystem.autoBuilder(eventMap).fullAuto(pieceBottomGroup);
      }
      private Command threePieceTopBlue(){
        List<PathPlannerTrajectory> pieceTopGroup = PathPlanner.loadPathGroup(
          "3 Piece Top Blue", new PathConstraints(4, 3));
        return swerveSubsystem.autoBuilder(eventMap).fullAuto(pieceTopGroup);
      }
      private Command onePlusParkBottomBlue(){
        List<PathPlannerTrajectory> parkBottomGroup = PathPlanner.loadPathGroup(
          "1 + Park Bottom Blue", new PathConstraints(.5, .5));
        return swerveSubsystem.autoBuilder(eventMap).fullAuto(parkBottomGroup);
      }

      private Command onePlusParkTopBlue(){
        List<PathPlannerTrajectory> parkTopGroup = PathPlanner.loadPathGroup(
          "1 + Park Top Blue", new PathConstraints(Constants.AutoConstants.maxSpeedMetersPerSecond, Constants.AutoConstants.maxAccelerationMetersPerSecondSquared));
        return swerveSubsystem.autoBuilder(eventMap).fullAuto(parkTopGroup);
      }
      private Command twoParkTopBlue(){
        List<PathPlannerTrajectory> twoParkTopGroup = PathPlanner.loadPathGroup(
          "2 + Park Top Blue", new PathConstraints(4, 3));
        return swerveSubsystem.autoBuilder(eventMap).fullAuto(twoParkTopGroup);
      }

      private Command goesFiveFeet(){
        List<PathPlannerTrajectory> fiveFeetGroup = PathPlanner.loadPathGroup(
          "GoesFiveFeet", new PathConstraints(1.0, .5));
        return swerveSubsystem.autoBuilder(eventMap).fullAuto(fiveFeetGroup).andThen(swerveSubsystem.driveCommand(() -> 0, ()-> 0, () -> 0, false, false, false));
      }

      private static Command run(Command ... commands) {
        return new ParallelCommandGroup(commands);
      }
}
