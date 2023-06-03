// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.HashMap;
import java.util.List;

/** Add your docs here. */
public class AutoContainer {
  public Command getAuto(IntakeSubsystem intakeSubsystem, SwerveSubsystem swerveSubsystem) {
    List<PathPlannerTrajectory> pathGroup =
        PathPlanner.loadPathGroup("FullAuto", new PathConstraints(4, 3));
    HashMap<String, Command> eventMap = new HashMap<>();

    eventMap.put("intake", new RunCommand(() -> intakeSubsystem.runCommand(), intakeSubsystem));
    return swerveSubsystem.autoBuilder(eventMap).fullAuto(pathGroup);
  }
}
