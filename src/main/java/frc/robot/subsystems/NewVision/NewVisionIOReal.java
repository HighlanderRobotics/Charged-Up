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
        
    }

}
