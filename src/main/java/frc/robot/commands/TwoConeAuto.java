package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PlacingSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TwoConeAuto extends SequentialCommandGroup {
    
    public TwoConeAuto(SwerveSubsystem swerveSubsystem,
    IntakeSubsystem intakeSubsystem,
    PlacingSubsystem placingSubsystem){
        SmartDashboard.putNumber("start heading", 
            PathPlanner.loadPath("firstcone", 3, 3).getInitialPose().getRotation().getDegrees());
    addCommands(
        new InstantCommand( () -> {
                swerveSubsystem.zeroGyro(0);
                swerveSubsystem.resetOdometry(PathPlanner.loadPath("firstcone", 3, 3)
                .getInitialPose());
            }, swerveSubsystem),
        // new RunCommand(() -> {placingSubsystem.activate();}, placingSubsystem).withTimeout(2.74),
        //max velo = 2; max acceleration = 1;
        swerveSubsystem.followPathCommand(PathPlanner.loadPath("firstcone", 2.0, 1.0)),
            // .alongWith(new RunCommand(() -> {intakeSubsystem.activate();}, intakeSubsystem)),

        swerveSubsystem.followPathCommand(PathPlanner.loadPath("secondpart", 2.0, 1.0))
        // new RunCommand(() -> {placingSubsystem.activate();}, placingSubsystem)
    );
    
        
    
    }
}
