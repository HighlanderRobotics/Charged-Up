// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.NewVision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;

public class NewVisionSubsystem extends SubsystemBase {
  /** Creates a new NewVisionSubsystem. */
  AprilTagFieldLayout fieldLayout;
  PhotonPoseEstimator estimator;

  /** If shuffleboard should be used--important for unit testing. */
  private static boolean useShuffleboard = true;

  private final ShuffleboardLayout cameraStatusList =
      Shuffleboard.getTab("DriverView")
          .getLayout("photonCameras", BuiltInLayouts.kList)
          .withPosition(11, 0)
          .withSize(2, 3);

  private double lastDetection = 0;
  
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    if (estimator == null) {
        // The field layout failed to load, so we cannot estimate poses.
        return Optional.empty();
    }
    estimator.setReferencePose(prevEstimatedRobotPose);
    return estimator.update();
}

  public NewVisionSubsystem() {
    try {
      fieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      System.err.println("Failed to load field layout.");
      e.printStackTrace();
      return;
    }

      var camera = new PhotonCamera(Vision.visionSource.name);
      var estimator =
          new PhotonPoseEstimator(
              fieldLayout,
              PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
              camera,
              Vision.visionSource.robotToCamera);
      estimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
      cameraStatusList.addBoolean(Vision.visionSource.name, camera::isConnected);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
