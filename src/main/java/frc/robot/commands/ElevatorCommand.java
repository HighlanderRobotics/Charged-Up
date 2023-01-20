// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Instant;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
    addCommands(swerveSubsystem.followPathCommand(swerveSubsystem.getPathToPoint(new PathPoint(new Translation2d(1.72, 2.72), 
    Rotation2d.fromDegrees(-4.40), Rotation2d.fromDegrees(180)))), 
    new InstantCommand(() -> elevatorSubsystem.extend(), elevatorSubsystem), 
    new InstantCommand(() -> elevatorSubsystem.place(), elevatorSubsystem), 
    new InstantCommand(() -> elevatorSubsystem.retract(), elevatorSubsystem));
  }
}
