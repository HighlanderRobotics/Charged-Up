// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Add your docs here. */
public class VisionIOSimApriltags implements VisionIO {
  SimVisionSystem sim =
      new SimVisionSystem("sim", 77.6, Constants.leftCameraToRobot.inverse(), 4.0, 320, 240, 10);

  PhotonCamera camera = new PhotonCamera("sim");

  public VisionIOSimApriltags() {
    try {
      sim.addVisionTargets(
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile));
    } catch (Exception e) {
      System.err.println("apriltag layout failed :(((((");
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs, Pose3d robotPose) {
    sim.processFrame(robotPose);
    var targets = camera.getLatestResult().getTargets();

    inputs.tags.clear();
    for (PhotonTrackedTarget target : targets) {
      inputs.tags.add((long) target.getFiducialId());
    }

    inputs.timestamp = camera.getLatestResult().getTimestampSeconds();
    inputs.timeSinceLastTimestamp = camera.getLatestResult().getLatencyMillis();
    inputs.lastPose = new Pose3d();
    inputs.cornersX = new double[targets.size() * 4];
    inputs.cornersY = new double[targets.size() * 4];
    for (int i = 0; i < targets.size(); i++) {
      for (int j = 0; j < 4; j++) {
        inputs.cornersY[(i * 4) + j] = targets.get(i).getDetectedCorners().get(j).y;
        inputs.cornersX[(i * 4) + j] = targets.get(i).getDetectedCorners().get(j).x;
      }
    }
  }
}
