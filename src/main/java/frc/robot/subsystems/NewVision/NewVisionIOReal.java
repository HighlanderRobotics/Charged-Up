// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.NewVision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
public class NewVisionIOReal implements NewVisionIO {
    PhotonCamera camera = new PhotonCamera("camera");

    @Override
    public void updateInputs(NewVisionIOInputs inputs, Pose3d robotPose) {
        var latestResult = camera.getLatestResult();

        inputs.timestamp = 0.0;
        inputs.timeSinceLastTimestamp = 0.0;
        inputs.targets = latestResult.targets;
        for (int i = 0; i < inputs.targets.size(); i++) {
            for (int j = 0; j < 4; j++) {
              NewVisionIOInputs.detectedCornersX[(i * 4) + j] = inputs.targets.get(i).getDetectedCorners().get(j).y;
              NewVisionIOInputs.detectedCornersY[(i * 4) + j] = inputs.targets.get(i).getDetectedCorners().get(j).x;
            }
          }
        
    }

}
