// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorCommand extends SequentialCommandGroup {
  /** Creates a new ElevatorCommand. */
  public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, SwerveSubsystem swerveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      swerveSubsystem.followPathCommand(
        swerveSubsystem.getPathToPoint(swerveSubsystem.getNearestGoal(new Pose2d()))),
      new InstantCommand(() -> elevatorSubsystem.extendElevator(), elevatorSubsystem), 
      new WaitUntilCommand(() -> elevatorSubsystem.isAtSetpoint()),
      new InstantCommand(() -> elevatorSubsystem.releaseElevator(), elevatorSubsystem), 
      new InstantCommand(() -> elevatorSubsystem.retractElevator(), elevatorSubsystem));
      
  
  }
  
  public static void main (String[] args) { //trying to test this but it's asking for the c++ runtime lol
    SwerveSubsystem test = new SwerveSubsystem();
    PathPoint output = test.getNearestGoal(new Pose2d(new Translation2d(2, 2), Rotation2d.fromDegrees(0)));
    System.out.println(output);
    
  }
}
