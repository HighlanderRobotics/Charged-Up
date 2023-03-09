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
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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

        eventMap.put("Score", new ScoringCommand(
          ScoringLevels.L3, 
          () -> 0, 
          elevatorSubsystem, 
          swerveSubsystem, 
          grabberSubsystem, 
          superstructureSubsystem).asProxy().andThen(
            new InstantCommand(() -> elevatorSubsystem.getDefaultCommand().schedule()),
            new PrintCommand("bbbbbbbb"), 
            new InstantCommand(() -> {}, swerveSubsystem),
            new WaitCommand(1.0),
            new PrintCommand("ccccccccccccccccccccccc")));
        eventMap.put("Score L3", new ScoringCommand(ScoringLevels.L3, () -> 0, elevatorSubsystem, swerveSubsystem, grabberSubsystem, superstructureSubsystem));
        eventMap.put("Score No Aim", 
          new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L3))
          .alongWith(swerveSubsystem.setGamePieceOverride(true))
          .andThen(superstructureSubsystem.scoreNoAim().asProxy()));
        eventMap.put("Score L3 No Aim", 
          new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L3))
          .alongWith(swerveSubsystem.setGamePieceOverride(true))
          .andThen(superstructureSubsystem.scoreNoAim().asProxy()));
        eventMap.put("Score L2 No Aim", 
          new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L2))
          .alongWith(swerveSubsystem.setGamePieceOverride(true))
          .andThen(superstructureSubsystem.scoreNoAim().asProxy()));
        eventMap.put("Score L2 No Aim Cube", 
          new InstantCommand(() -> swerveSubsystem.setLevel(ScoringLevels.L2))
          .alongWith(swerveSubsystem.setGamePieceOverride(false))
          .andThen(superstructureSubsystem.scoreNoAim().asProxy()));
        eventMap.put("Test Wait", new WaitCommand(2.0));
        eventMap.put("Balance", swerveSubsystem.autoBalance());
        eventMap.put("Intake", run(
          intakeSubsystem.runCommand(), 
          routingSubsystem.runCommand(), 
          grabberSubsystem.intakeOpenCommand(),
          armSubsystem.runToRoutingCommand()).withTimeout(4.0).asProxy()); 
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

        chooser.setDefaultOption("none", new InstantCommand(() -> {}));
            //"Two Cone Auto", 
            //new TwoConeAuto(swerveSubsystem, intakeSubsystem));
        chooser.addOption("2 + Park Top Blue", twoParkTopBlue());
        chooser.addOption("1.5 + Park Top Blue", onePlusParkTopBlue());
        chooser.addOption("1 + Park Middle Blue", onePlusParkMiddle());
        chooser.addOption("2 Park Top Blue", twoParkTopBlue());

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
          "2 + Park Top Blue", new PathConstraints(Constants.AutoConstants.maxSpeedMetersPerSecond, Constants.AutoConstants.maxAccelerationMetersPerSecondSquared));
        return swerveSubsystem.autoBuilder(eventMap).fullAuto(twoParkTopGroup);
      }

      private Command twoTopBlue(){
        List<PathPlannerTrajectory> twoParkTopGroup = PathPlanner.loadPathGroup(
          "2 Top Blue", new PathConstraints(Constants.AutoConstants.maxSpeedMetersPerSecond, Constants.AutoConstants.maxAccelerationMetersPerSecondSquared));
        return swerveSubsystem.autoBuilder(eventMap).fullAuto(twoParkTopGroup);
      }

      private Command onePlusParkMiddle() {
        List<PathPlannerTrajectory> twoParkTopGroup = PathPlanner.loadPathGroup(
          "Middle 1 + Balance", new PathConstraints(Constants.AutoConstants.maxSpeedMetersPerSecond, Constants.AutoConstants.maxAccelerationMetersPerSecondSquared));
        return swerveSubsystem.autoBuilder(eventMap).fullAuto(twoParkTopGroup);
      }

      private Command onePlusParkBottom() {
        List<PathPlannerTrajectory> twoParkTopGroup = PathPlanner.loadPathGroup(
          "1.5 + Park Bottom Blue", new PathConstraints(Constants.AutoConstants.maxSpeedMetersPerSecond, Constants.AutoConstants.maxAccelerationMetersPerSecondSquared));
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
