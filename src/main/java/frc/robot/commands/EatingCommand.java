// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.PathPointOpen;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EatingCommand extends SequentialCommandGroup {
  /** Creates a new EatingCommand. */
  public EatingCommand(
    SuperstructureSubsystem superstructureSubsystem,
    ElevatorSubsystem elevatorSubsystem, 
    ArmSubsystem armSubsystem, 
    SwerveSubsystem swerveSubsystem,
    GrabberSubsystem grabberSubsystem,
    LEDSubsystem ledSubsystem) 
   {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    PathPointOpen substationLocation = null;

    if (DriverStation.getAlliance() == Alliance.Blue) {
      substationLocation = Constants.blueSubstation;
    }
    if (DriverStation.getAlliance() == Alliance.Red) {
      substationLocation = Constants.redSubstation;
    }
    addCommands(
      swerveSubsystem.followPathCommand(
        swerveSubsystem.getPathToPoint(substationLocation)).alongWith(
          ledSubsystem.setSolidCommand(new Color8Bit(20, 107, 241))
        ),
      new PrintCommand("finished path"),
      swerveSubsystem.headingLockDriveCommand(
        () -> 0, () -> 0, () -> swerveSubsystem.getNearestGoal().getRotation2d().getRadians(), 
        false, false).alongWith(
          ledSubsystem.setSolidCommand(new Color8Bit(13, 240, 78))).alongWith(
          superstructureSubsystem.waitExtendToInches(Constants.ScoringLevels.humanPlayerLevel)), 
      // ElevatorSubsystem.extendCommand(elevatorSubsystem, armSubsystem, Level.HUMAN_PLAYER, false),
      //we're only picking up cones from the substations
      new WaitUntilCommand(() -> elevatorSubsystem.isAtGoal() && armSubsystem.isAtSetpoint()),
      grabberSubsystem.closeCommand()
    );
  }
}
