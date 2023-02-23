// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoringCommand extends SequentialCommandGroup {
  /** Creates a new ScoringCommand. */
  public ScoringCommand(
    double level,
    ElevatorSubsystem elevatorSubsystem, 
    ArmSubsystem armSubsystem, 
    SwerveSubsystem swerveSubsystem,
    GrabberSubsystem grabberSubsystem,
    SuperstructureSubsystem superstructureSubsystem) {
    // LEDSubsystem ledSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(
    new PrintCommand("scoring sequence woo" + swerveSubsystem.getPose().toString()),
      swerveSubsystem.followPathCommand(
        //swerveSubsystem.getPathBetweenTwoPoints(
        //  new PathPoint(new Translation2d(5, 5), new Rotation2d()), swerveSubsystem.getNearestGoal())),
       
        swerveSubsystem.getPathToPoint(swerveSubsystem.getNearestGoal())),//.alongWith(
         // ledSubsystem.setSolidCommand(new Color8Bit(20, 107, 241))),
      new PrintCommand("finished path"),
      swerveSubsystem.headingLockDriveCommand(
        () -> 0, () -> 0, () -> swerveSubsystem.getNearestGoal().getRotation2d().getRadians(), 
        false, false)//.alongWith(
          //ledSubsystem.setSolidCommand(new Color8Bit(13, 240, 78)))
          .alongWith(
          superstructureSubsystem.waitExtendToInches(level)),
      // elevatorSubsystem.extendCommand(
      //   swerveSubsystem.checkIfConeGoal(swerveSubsystem.getNearestGoal())),
      //   elevatorSubsystem, armSubsystem, elevatorSubsystem.getLevel(), 
      new WaitUntilCommand(() -> elevatorSubsystem.isAtSetpoint()), //&& armSubsystem.isAtSetpoint()),
      grabberSubsystem.openCommand()
    );
  }
}
