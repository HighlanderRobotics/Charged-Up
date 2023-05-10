// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants.PoseEstimator;
import frc.robot.Constants.Vision;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOApriltags implements VisionIO {
  private class CameraEstimator {
    public PhotonPoseEstimator estimator;
    public PhotonCamera camera;

    public CameraEstimator(PhotonCamera camera, PhotonPoseEstimator estimator) {
      this.estimator = estimator;
      this.camera = camera;
    }
  }
  /** If shuffleboard should be used--important for unit testing. */
  private static boolean useShuffleboard = true;

  private final ShuffleboardLayout cameraStatusList =
      Shuffleboard.getTab("DriverView")
          .getLayout("photonCameras", BuiltInLayouts.kList)
          .withPosition(11, 0)
          .withSize(2, 3);

  private final List<CameraEstimator> estimators = new ArrayList<>();

  private AprilTagFieldLayout fieldLayout;

  private double lastDetection = 0;

  /** Creates a new VisionSubsystem. */
  public VisionIOApriltags() {
    // loading the 2023 field arrangement
    try {
      fieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      System.err.println("Failed to load field layout.");
      e.printStackTrace();
      return;
    }

    for (Vision.VisionSource visionSource : Vision.VISION_SOURCES) {
      var camera = new PhotonCamera(visionSource.name);
      var estimator =
          new PhotonPoseEstimator(
              fieldLayout,
              PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
              camera,
              visionSource.robotToCamera);
      estimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
      cameraStatusList.addBoolean(visionSource.name, camera::isConnected);
      estimators.add(new CameraEstimator(camera, estimator));
    }

    if (useShuffleboard)
      cameraStatusList.addString(
          "time since apriltag detection",
          () -> String.format("%3.0f seconds", Timer.getFPGATimestamp() - lastDetection));
  }

  private static boolean ignoreFrame(PhotonPipelineResult frame) {
    if (!frame.hasTargets() || frame.getTargets().size() > PoseEstimator.MAX_FRAME_FIDS)
      return true;

    boolean possibleCombination = false;
    List<Integer> ids = frame.targets.stream().map(t -> t.getFiducialId()).toList();
    for (Set<Integer> possibleFIDCombo : PoseEstimator.POSSIBLE_FRAME_FID_COMBOS) {
      possibleCombination = possibleFIDCombo.containsAll(ids);
      if (possibleCombination) break;
    }
    if (!possibleCombination) System.out.println("Ignoring frame with FIDs: " + ids);
    return !possibleCombination;
  }

  @Override
  public List<VisionMeasurement> getMeasurement(Pose2d prevEstimatedRobotPose) {
    if (fieldLayout == null) {
      return List.of();
    }

    List<VisionMeasurement> estimations = new ArrayList<>();

    for (CameraEstimator cameraEstimator : estimators) {
      var estimator = cameraEstimator.estimator;
      var camera = cameraEstimator.camera;

      if (ignoreFrame(camera.getLatestResult())) continue;

      estimator.setReferencePose(prevEstimatedRobotPose);
      var optEstimation = estimator.update();
      if (optEstimation.isEmpty()) continue;
      var estimation = optEstimation.get();
      double smallestDistance = Double.POSITIVE_INFINITY;
      for (var target : estimation.targetsUsed) {
        var t3d = target.getBestCameraToTarget();
        var distance =
            Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
        if (distance < smallestDistance) smallestDistance = distance;
      }
      double poseAmbiguityFactor =
          estimation.targetsUsed.size() != 1
              ? 1
              : Math.max(
                  1,
                  (estimation.targetsUsed.get(0).getPoseAmbiguity()
                          + PoseEstimator.POSE_AMBIGUITY_SHIFTER)
                      * PoseEstimator.POSE_AMBIGUITY_MULTIPLIER);
      double confidenceMultiplier =
          Math.max(
              1,
              (Math.max(
                          1,
                          Math.max(0, smallestDistance - PoseEstimator.NOISY_DISTANCE_METERS)
                              * PoseEstimator.DISTANCE_WEIGHT)
                      * poseAmbiguityFactor)
                  / (1
                      + ((estimation.targetsUsed.size() - 1) * PoseEstimator.TAG_PRESENCE_WEIGHT)));
      // System.out.println(
      //     String.format(
      //         "with %d tags at smallest distance %f and pose ambiguity factor %f, confidence
      // multiplier %f",
      //         estimation.targetsUsed.size(),
      //         smallestDistance,
      //         poseAmbiguityFactor,
      //         confidenceMultiplier));
      estimations.add(
          new VisionMeasurement(
              estimation,
              PoseEstimator.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier)));
    }

    if (!estimations.isEmpty()) {
      lastDetection = Timer.getFPGATimestamp();
    }

    return estimations;
  }
}
