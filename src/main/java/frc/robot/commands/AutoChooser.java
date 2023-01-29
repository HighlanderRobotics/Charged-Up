package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PlacingSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoChooser {

    
    List<PathPlannerTrajectory> twoConeGroup = PathPlanner.loadPathGroup
    ("TwoCone", new PathConstraints(4, 3));
    HashMap<String, Command> eventMap = new HashMap<>();
    
    

    SendableChooser<Command> chooser = new SendableChooser<Command>();
    SwerveSubsystem swerveSubsystem;
    IntakeSubsystem intakeSubsystem;
    PlacingSubsystem placingSubsystem;  
    public AutoChooser(SwerveSubsystem swerveSubsystem,
    IntakeSubsystem intakeSubsystem,
    PlacingSubsystem placingSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        this.swerveSubsystem = swerveSubsystem; 
        chooser.setDefaultOption("First cone", new TwoConeAuto
            (swerveSubsystem, intakeSubsystem, placingSubsystem));
        chooser.addOption("NONE", new PrintCommand("owo"));

        SmartDashboard.putData(chooser);

        for (int x=0; x<2; x++) {
            eventMap.put("Place", new InstantCommand(()-> placingSubsystem.activate()));
            eventMap.put("Intake", new InstantCommand(()-> intakeSubsystem.activate()));
        }
    
    
        swerveSubsystem.autoBuilder(eventMap);
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

      private Command twoConeAuto(){
        return new SequentialCommandGroup(
        runPlacer(),
        new InstantCommand(() -> swerveSubsystem.zeroGyro(0)),  
        new InstantCommand(() -> swerveSubsystem.resetOdometry(PathPlanner.loadPath("firstcone",
             8.0, 5.0).getInitialPose())),
        runIntake(),
        new InstantCommand(() -> swerveSubsystem.resetOdometry(PathPlanner.loadPath("secondpart",
        8.0, 5.0).getInitialPose())),
        runPlacer()

        );
      }
}
