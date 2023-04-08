// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.math;

import edu.wpi.first.math.geometry.CoordinateAxis;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
public class HRCoordinateSystem extends CoordinateSystem {
    public HRCoordinateSystem(CoordinateAxis positiveX, CoordinateAxis positiveY, CoordinateAxis positiveZ) {
        super(positiveX, positiveY, positiveZ);
      }

    public static CoordinateSystem fromPose(Pose3d pose) {
        double pitch = pose.getRotation().getY();
        CoordinateAxis positiveX = new CoordinateAxis(0 , 0 , 0);
        CoordinateAxis positiveY = new CoordinateAxis(-0 , 0 , 0);
        CoordinateAxis positiveZ = new CoordinateAxis(pose.getX(), pose.getY(), 1);
        return new CoordinateSystem(positiveX, positiveY, positiveZ);
    }
}
