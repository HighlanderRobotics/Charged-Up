// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class PathPointOpen extends PathPoint {

    public PathPointOpen(Translation2d position, Rotation2d heading) {
        super(position, heading);
        //TODO Auto-generated constructor stub
    }
    public Translation2d getTranslation2d () {
        return position;
    }
}
