// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ScoringLevels;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoringCommand extends SequentialCommandGroup {
  /** Creates a new ScoringCommand. */
  public ScoringCommand(
    ElevatorSubsystem.ScoringLevels level,
    DoubleSupplier adjustmentSupplier,
    ElevatorSubsystem elevatorSubsystem, 
    SwerveSubsystem swerveSubsystem,
    GrabberSubsystem grabberSubsystem,
    SuperstructureSubsystem superstructureSubsystem) {
    // LEDSubsystem ledSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(
      new PrintCommand("scoring sequence woo" + swerveSubsystem.getPose().toString()),
      swerveSubsystem.poseLockDriveCommand(
        () -> swerveSubsystem.getNearestGoal().getTranslation2d().getX(), 
        () -> swerveSubsystem.getNearestGoal().getTranslation2d().getY(), 
        () -> swerveSubsystem.getNearestGoal().getRotation2d().getRadians(), 
        true, false)
      // swerveSubsystem.driveCommand(() -> adjustmentSupplier.getAsDouble(), () -> 0, () -> 0, false, false)
        .alongWith(
          new WaitUntilCommand(() -> {return swerveSubsystem.getNearestGoalDistance() < 0.05;})//.alongWith(
            .andThen(
              grabberSubsystem.closeCommand(),
                //ledSubsystem.setSolidCommand(new Color8Bit(13, 240, 78)))
              new PrintCommand(level + " level"),
              new PrintCommand(swerveSubsystem.checkIfConeGoal(swerveSubsystem.getNearestGoal()) + " nearest goal is cone"),
                superstructureSubsystem.waitExtendToGoal(level)
                .andThen(
                    new PrintCommand("extended elevator"),
                    new WaitCommand(0.25),
                    new ConditionalCommand(
                      grabberSubsystem.outakeNeutralCommand(), 
                      new ConditionalCommand(
                        grabberSubsystem.openCommand(), 
                        grabberSubsystem.outakeOpenCommand(), 
                        () -> swerveSubsystem.nearestGoalIsCone), 
                      () -> swerveSubsystem.checkIfConeGoal(swerveSubsystem.getNearestGoal()) && level == ScoringLevels.L3)
                  )
                  // .withTimeout(1)
                  )
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
            )
    );
  }
}
