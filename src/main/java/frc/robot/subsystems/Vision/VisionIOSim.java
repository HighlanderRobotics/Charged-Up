// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;
import java.io.IOException;
import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;

/** Add your docs here. */
public class VisionIOSim implements VisionIO {
  SimVisionSystem sim =
      new SimVisionSystem("camera", 70, Constants.leftCameraToRobot.inverse(), 9000, 1280, 800, 10);
  PhotonCamera camera = new PhotonCamera("camera");

  public VisionIOSim() {
    try {
      var field = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      sim.addVisionTargets(field);
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs, Pose3d robotPose) {
    sim.processFrame(robotPose);
    var result = camera.getLatestResult();

    inputs.timestamp = result.getTimestampSeconds();
    inputs.timeSinceLastTimestamp = 0.0;
    inputs.targets = result.targets;
  }
}
