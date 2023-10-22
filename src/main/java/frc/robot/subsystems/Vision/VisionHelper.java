// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class VisionHelper {
    private static final PhotonCamera camera = new PhotonCamera(Constants.Vision.visionSource.name);

/**
     * Poll data from the configured cameras and update the estimated position of the robot. Returns
     * empty if there are no cameras set or no targets were found from the cameras.
     *
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
     *     the estimate
     */
    public static Optional<EstimatedRobotPose> update(
        PhotonPipelineResult result, 
        AprilTagFieldLayout fieldTags,
        PoseStrategy primaryStrategy,
        PoseStrategy multiTagFallbackStrategy) {
        
        double poseCacheTimestampSeconds = -1;
        if (camera == null) {
            DriverStation.reportError("[PhotonPoseEstimator] Missing camera!", false);
            return Optional.empty();
        }

        // Time in the past -- give up, since the following if expects times > 0
        if (result.getTimestampSeconds() < 0) {
            return Optional.empty();
        }

        // If the pose cache timestamp was set, and the result is from the same timestamp, return an
        // empty result
        if (poseCacheTimestampSeconds > 0
                && Math.abs(poseCacheTimestampSeconds - result.getTimestampSeconds()) < 1e-6) {
            return Optional.empty();
        }

        // Remember the timestamp of the current result used
        poseCacheTimestampSeconds = result.getTimestampSeconds();

        // If no targets seen, trivial case -- return empty result
        if (!result.hasTargets()) {
            return Optional.empty();
        }

        Optional<EstimatedRobotPose> estimatedPose;
        switch (primaryStrategy) {
            case LOWEST_AMBIGUITY:
                estimatedPose = lowestAmbiguityStrategy(result, fieldTags);
                break;
            case MULTI_TAG_PNP:
                estimatedPose = multiTagPNPStrategy(result, fieldTags, primaryStrategy, multiTagFallbackStrategy);
                break;
            default:
                DriverStation.reportError(
                    "[PhotonPoseEstimator] Unknown Position Estimation Strategy!", false);
                return Optional.empty();
        }

        return estimatedPose;
}

    private static Optional<EstimatedRobotPose> multiTagPNPStrategy(
        PhotonPipelineResult result,
        AprilTagFieldLayout fieldTags,
        PoseStrategy primaryStrategy,
        PoseStrategy multiTagFallbackStrategy) {
        // Arrays we need declared up front
        var visCorners = new ArrayList<TargetCorner>();
        var knownVisTags = new ArrayList<AprilTag>();
        var fieldToCams = new ArrayList<Pose3d>();
        var fieldToCamsAlt = new ArrayList<Pose3d>();
        double[] visCornersX = new double[4];
        double[] visCornersY = new double[4];

        if (result.getTargets().size() < 2) {
            // Run fallback strategy instead
            return update(result, fieldTags, multiTagFallbackStrategy, multiTagFallbackStrategy);
        }

        for (var target : result.getTargets()) {
            visCorners.addAll(target.getDetectedCorners());

            var tagPoseOpt = fieldTags.getTagPose(target.getFiducialId());
            if (tagPoseOpt.isEmpty()) {
                DriverStation.reportError(
                    "[PhotonPoseEstimator] Tried to get pose of unknown AprilTag: " + target.getFiducialId(), false);
                continue;
            }

            var tagPose = tagPoseOpt.get();

            // actual layout poses of visible tags -- not exposed, so have to recreate
            knownVisTags.add(new AprilTag(target.getFiducialId(), tagPose));

            fieldToCams.add(tagPose.transformBy(target.getBestCameraToTarget().inverse()));
            fieldToCamsAlt.add(tagPose.transformBy(target.getAlternateCameraToTarget().inverse()));
        }
        
        var cameraMatrixOpt = camera.getCameraMatrix();
        var distCoeffsOpt = camera.getDistCoeffs();
        boolean hasCalibData = cameraMatrixOpt.isPresent() && distCoeffsOpt.isPresent();

        for (TargetCorner corner : visCorners) {
            visCornersX[visCorners.indexOf(corner)] = corner.x;
            visCornersY[visCorners.indexOf(corner)] = corner.y;
        }
        // multi-target solvePNP
        if (hasCalibData) {
            var cameraMatrix = cameraMatrixOpt.get();
            var distCoeffs = distCoeffsOpt.get();
            var pnpResults =
                VisionEstimation.estimateCamPosePNP(cameraMatrix, distCoeffs, visCorners, knownVisTags);
            var best =
                new Pose3d()
                    .plus(pnpResults.best) // field-to-camera
                    .plus(Constants.Vision.visionSource.robotToCamera.inverse()); // field-to-robot
            // var alt = new Pose3d()
            // .plus(pnpResults.alt) // field-to-camera
            // .plus(robotToCamera.inverse()); // field-to-robot
            var estimatedRobotPose = new EstimatedRobotPose(best, result.getTimestampSeconds(), result.getTargets());
            return Optional.of(
                estimatedRobotPose);
        } else {
            return lowestAmbiguityStrategy(result, fieldTags);
        }
    }

    /**
     * Return the estimated position of the robot with the lowest position ambiguity from a List of
     * pipeline results.
     *
     * @param result pipeline result
     * @return the estimated position of the robot in the FCS and the estimated timestamp of this
     *     estimation.
     */
    private static Optional<EstimatedRobotPose> lowestAmbiguityStrategy(PhotonPipelineResult result, AprilTagFieldLayout fieldTags) {
        PhotonTrackedTarget lowestAmbiguityTarget = null;

        double lowestAmbiguityScore = 10;

        for (PhotonTrackedTarget target : result.targets) {
            double targetPoseAmbiguity = target.getPoseAmbiguity();
            // Make sure the target is a Fiducial target.
            if (targetPoseAmbiguity != -1 && targetPoseAmbiguity < lowestAmbiguityScore) {
                lowestAmbiguityScore = targetPoseAmbiguity;
                lowestAmbiguityTarget = target;
            }
        }

        // Although there are confirmed to be targets, none of them may be fiducial
        // targets.
        if (lowestAmbiguityTarget == null) return Optional.empty();

        int targetFiducialId = lowestAmbiguityTarget.getFiducialId();

        Optional<Pose3d> targetPosition = fieldTags.getTagPose(targetFiducialId);

        if (targetPosition.isEmpty()) {
            DriverStation.reportError(
                    "[PhotonPoseEstimator] Tried to get pose of unknown AprilTag: " + targetFiducialId, false);
            return Optional.empty();
        }
        var estimatedRobotPose = new EstimatedRobotPose(
            targetPosition
                .get()
                .transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
                .transformBy(Constants.Vision.visionSource.robotToCamera.inverse()),
                    result.getTimestampSeconds(),
                    result.getTargets());
        return Optional.of(estimatedRobotPose);
    }
}