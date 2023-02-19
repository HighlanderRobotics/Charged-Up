// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Level;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoringCommand extends SequentialCommandGroup {
  /** Creates a new ScoringCommand. */
  public ScoringCommand(
    Level level, 
    ElevatorSubsystem elevatorSubsystem, 
    ArmSubsystem armSubsystem, 
    SwerveSubsystem swerveSubsystem,
    GrabberSubsystem grabberSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(
      swerveSubsystem.followPathCommand(
        swerveSubsystem.getPathToPoint(swerveSubsystem.getNearestGoal())),
      new PrintCommand("finished path"),
      swerveSubsystem.headingLockDriveCommand(
        () -> 0, () -> 0, () -> swerveSubsystem.getNearestGoal().getRotation2d().getRadians(), 
        false, false),
      ElevatorSubsystem.extendCommand(
        elevatorSubsystem, armSubsystem, elevatorSubsystem.getLevel(), 
        swerveSubsystem.checkIfConeGoal(swerveSubsystem.getNearestGoal())),//TODO: find if actually is cone
      new WaitUntilCommand(() -> elevatorSubsystem.isAtSetpoint() && armSubsystem.isAtSetpoint()),
      grabberSubsystem.openCommand()
    );
  }
}
