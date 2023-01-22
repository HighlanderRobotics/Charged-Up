package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

public class TwoConeAuto extends SequentialCommandGroup {
    
    public TwoConeAuto(SwerveSubsystem swerveSubsystem,
    IntakeSubsystem intakeSubsystem,
    PlacingSubsystem placingSubsytem){

    addCommands(
        
        
        new RunCommand(() -> {placingSubsytem.activate();}, placingSubsystem),
                
        //max velo = 2; max acceleration = 1;
        swerveSubsystem.followPathCommand(PathPlanner.loadPath("firstcone", 2.0, 1.0)),

        new RunCommand(() -> {intakeSubsystem.activate();}, intakeSubsystem),

        swerveSubsystem.followPathCommand(PathPlanner.loadPath("secondpart", 2.0, 1.0)),

        new RunCommand(() -> {placingSubsystem.activate();}, placingSubsystem)

        
    );

        
    
    }
}
