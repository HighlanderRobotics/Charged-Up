package frc.robot.commands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PlacingSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoChooser {

    
    

    SendableChooser<Command> chooser = new SendableChooser<Command>();
    SwerveSubsystem swerveSubsystem;
    IntakeSubsystem intakeSubsystem;
    PlacingSubsystem placingSubsystem;
    HashMap<String, Command> eventMap = new HashMap<>();
    
    
    public AutoChooser(SwerveSubsystem swerveSubsystem,
    IntakeSubsystem intakeSubsystem,
    PlacingSubsystem placingSubsystem){
      
        this.intakeSubsystem = intakeSubsystem;
        this.swerveSubsystem = swerveSubsystem; 
        chooser.setDefaultOption(
            "Two Cone Auto", 
            new TwoConeAuto(swerveSubsystem, intakeSubsystem, placingSubsystem));
        chooser.addOption("2 + Park Middle Blue", parkMiddleBlue());
        chooser.addOption("NONE", new PrintCommand("owo"));

        SmartDashboard.putData("autotester", chooser);

        //List<PathPlannerTrajectory> twoConeGroup = PathPlanner.loadPathGroup
    //("TwoCone", new PathConstraints(4, 3));
    //HashMap<String, Command> Constants.eventMap = new HashMap<>();
    
    
      
    }
    
    
    

    public Command getAutoCommand(){
      return chooser.getSelected();
    }

    private Command runIntake() {
        return new ParallelCommandGroup(
          new WaitCommand(1.0).andThen(new RunCommand(() -> 
          {
            intakeSubsystem.activate(); 
            //intakeSubsystem.setIntakeRPM(4000);
          }, intakeSubsystem)));
      }

      private Command runPlacer() {
        return new InstantCommand(() -> placingSubsystem.activate());
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

      private Command PieceBottomBlue(){
        List<PathPlannerTrajectory> pieceBottomGroup = PathPlanner.loadPathGroup(
          "3 Piece Bottom Blue", new PathConstraints(4, 3));
        return swerveSubsystem.autoBuilder(eventMap).fullAuto(pieceBottomGroup);
      }
      private Command PieceTopBlue(){
        List<PathPlannerTrajectory> pieceTopGroup = PathPlanner.loadPathGroup(
          "3 Piece Top Blue", new PathConstraints(4, 3));
        return swerveSubsystem.autoBuilder(eventMap).fullAuto(pieceTopGroup);
      }
      private Command ParkBottomBlue(){
        List<PathPlannerTrajectory> parkBottomGroup = PathPlanner.loadPathGroup(
          "1+ Park Bottom Blue", new PathConstraints(4, 3));
        return swerveSubsystem.autoBuilder(eventMap).fullAuto(parkBottomGroup);
      }

      private Command ParkTopBlue(){
        List<PathPlannerTrajectory> parkTopGroup = PathPlanner.loadPathGroup(
          "1+ Park Top Blue", new PathConstraints(4, 3));
        return swerveSubsystem.autoBuilder(eventMap).fullAuto(parkTopGroup);
      }
      private Command twoParkTopBlue(){
        List<PathPlannerTrajectory> twoParkTopGroup = PathPlanner.loadPathGroup(
          "2+ Park Top Blue", new PathConstraints(4, 3));
        return swerveSubsystem.autoBuilder(eventMap).fullAuto(twoParkTopGroup);
      }

      
}
