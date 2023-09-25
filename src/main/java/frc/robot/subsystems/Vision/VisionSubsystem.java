// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.Vision.VisionIO.VisionIOInputs;

public class VisionSubsystem {
  VisionIOReal visionIOReal;
  VisionIOInputs visionIOInputs;
  
  public void updateVisionInputs(VisionIOInputs inputs, Pose3d pose) {
    visionIOReal.updateInputs(inputs, pose);
  }
  public VisionSubsystem() {}

}
