// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.VisionIO.VisionIOInputs;


public abstract class LoggedEstimator implements LoggableInputs {
  /** Creates a new LoggedEstimator. */

  private VisionIOInputs visionIOInputs = new VisionIOInputs();
  private AprilTagFieldLayout fieldTags;
  private PoseStrategy primaryStrategy = PoseStrategy.MULTI_TAG_PNP;
  private PoseStrategy multiTagFallbackStrategy = PoseStrategy.LOWEST_AMBIGUITY;
  private final PhotonCamera camera = new PhotonCamera(Constants.Vision.visionSource.name);
  private Transform3d robotToCamera;

  private Pose3d lastPose;
  private Pose3d referencePose;
  protected double poseCacheTimestampSeconds = -1;
  private final Set<Integer> reportedErrors = new HashSet<>();

  private PhotonPoseEstimator estimator = new PhotonPoseEstimator(
    fieldTags, primaryStrategy, camera, robotToCamera);

  public static void logPose3d(Pose3d pose3d, LogTable table, String name) {
    double rotation[] = new double[4];
    rotation[0] = pose3d.getRotation().getQuaternion().getW();
    rotation[1] = pose3d.getRotation().getQuaternion().getX();
    rotation[2] = pose3d.getRotation().getQuaternion().getY();
    rotation[3] = pose3d.getRotation().getQuaternion().getZ();
    table.put("rotation " + name, rotation);

    double translation[] = new double[3];
    translation[0] = pose3d.getTranslation().getX();
    translation[1] = pose3d.getTranslation().getY();
    translation[2] = pose3d.getTranslation().getZ();
    table.put("translation " + name, translation);
  }

  public Pose3d getLoggedPose3d(double[] translation, double[] rotation) {
      Pose3d pose3d =
          new Pose3d(
              new Translation3d(translation[0], translation[1], translation[2]),
              new Rotation3d(new Quaternion(rotation[0], rotation[1], rotation[2], rotation[3])));
      return pose3d;
    }

  /** Invalidates the pose cache. */
  private void invalidatePoseCache() {
    poseCacheTimestampSeconds = -1;
}

private void checkUpdate(Object oldObj, Object newObj) {
    if (oldObj != newObj && oldObj != null && !oldObj.equals(newObj)) {
        invalidatePoseCache();
    }
}

/**
 * Get the AprilTagFieldLayout being used by the PositionEstimator.
 *
 * @return the AprilTagFieldLayout
 */
public AprilTagFieldLayout getFieldTags() {
  return estimator.getFieldTags();
}

/**
 * Set the AprilTagFieldLayout being used by the PositionEstimator.
 *
 * @param fieldTags the AprilTagFieldLayout
 */
public void setFieldTags(AprilTagFieldLayout fieldTags) {
  estimator.setFieldTags(fieldTags);
}

/**
 * Get the Position Estimation Strategy being used by the Position Estimator.
 *
 * @return the strategy
 */
public PoseStrategy getPrimaryStrategy() {
  return estimator.getPrimaryStrategy();
}

/**
 * Set the Position Estimation Strategy used by the Position Estimator.
 *
 * @param strategy the strategy to set
 */
public void setPrimaryStrategy(PoseStrategy strategy) {
  estimator.setPrimaryStrategy(strategy);
}

/**
 * Set the Position Estimation Strategy used in multi-tag mode when only one tag can be seen. Must
 * NOT be MULTI_TAG_PNP
 *
 * @param strategy the strategy to set
 */
public void setMultiTagFallbackStrategy(PoseStrategy strategy) {
  estimator.setMultiTagFallbackStrategy(strategy);
}

/**
 * Return the reference position that is being used by the estimator.
 *
 * @return the referencePose
 */
public Pose3d getReferencePose(LogTable table, String name) {
  double[] translation = table.getDoubleArray("translation " + name, new double[3]);
  double[] rotation = table.getDoubleArray("rotation " + name, new double[4]);
  Pose3d referencePose = getLoggedPose3d(translation, rotation);
  return referencePose;
}

/**
 * Update the stored reference pose for use when using the <b>CLOSEST_TO_REFERENCE_POSE</b>
 * strategy.
 *
 * @param referencePose the referencePose to set
 */
public void setReferencePose(Pose3d referencePose, LogTable table) {
  logPose3d(referencePose, table, "reference pose");
  checkUpdate(this.referencePose, referencePose);
  this.referencePose = referencePose;
}

/**
 * Update the stored last pose. Useful for setting the initial estimate when using the
 * <b>CLOSEST_TO_LAST_POSE</b> strategy.
 *
 * @param lastPose the lastPose to set
 */
public void setLastPose(Pose3d lastPose, LogTable table, String name) {
  logPose3d(lastPose, table, "last pose " + name);
  this.lastPose = lastPose;
}

/**
 * Update the stored last pose. Useful for setting the initial estimate when using the
 * <b>CLOSEST_TO_LAST_POSE</b> strategy.
 *
 * @param lastPose the lastPose to set
 */
public void setLastPose(Pose2d lastPose, LogTable table, String name) {
    setLastPose(new Pose3d(lastPose), table, name);
}

/** @return The current transform from the center of the robot to the camera mount position */
public Transform3d getRobotToCameraTransform() {
    return robotToCamera;
}

/**
 * Useful for pan and tilt mechanisms and such.
 *
 * @param robotToCamera The current transform from the center of the robot to the camera mount
 *     position
 */
public void setRobotToCameraTransform(Transform3d robotToCamera) {
    this.robotToCamera = robotToCamera;
}

/**
 * Poll data from the configured cameras and update the estimated position of the robot. Returns
 * empty if there are no cameras set or no targets were found from the cameras.
 *
 * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
 *     the estimate
 */
public Optional<EstimatedRobotPose> update(LogTable table) {
  double latencyMillis = table.getDouble("latency", latencyMillis);
  List<PhotonTrackedTarget> targets = visionIOInputs.targets;
    if (camera == null) {
        DriverStation.reportError("[PhotonPoseEstimator] Missing camera!", false);
        return Optional.empty();
    }

    PhotonPipelineResult cameraResult = new PhotonPipelineResult(latencyMillis, targets);

    return update(cameraResult);
}

/**
 * Updates the estimated position of the robot. Returns empty if there are no cameras set or no
 * targets were found from the cameras.
 *
 * @param cameraResult The latest pipeline result from the camera
 * @return an EstimatedRobotPose with an estimated pose, and information about the camera(s) and
 *     pipeline results used to create the estimate
 */
public Optional<EstimatedRobotPose> update(PhotonPipelineResult cameraResult) {
  return estimator.update(cameraResult);
}

private Optional<EstimatedRobotPose> update(PhotonPipelineResult cameraResult, PoseStrategy strat, LogTable table) {
    Optional<EstimatedRobotPose> estimatedPose;
    switch (strat) {
        case LOWEST_AMBIGUITY:
            estimatedPose = lowestAmbiguityStrategy(cameraResult);
            break;
        case CLOSEST_TO_CAMERA_HEIGHT:
            estimatedPose = closestToCameraHeightStrategy(cameraResult);
            break;
        case CLOSEST_TO_REFERENCE_POSE:
            estimatedPose = closestToReferencePoseStrategy(cameraResult, referencePose);
            break;
        case CLOSEST_TO_LAST_POSE:
            setReferencePose(lastPose, table);
            estimatedPose = closestToReferencePoseStrategy(cameraResult, referencePose);
            break;
        case AVERAGE_BEST_TARGETS:
            estimatedPose = averageBestTargetsStrategy(cameraResult);
            break;
        case MULTI_TAG_PNP:
            estimatedPose = multiTagPNPStrategy(cameraResult, table);
            break;
        default:
            DriverStation.reportError(
                    "[PhotonPoseEstimator] Unknown Position Estimation Strategy!", false);
            return Optional.empty();
    }

    if (estimatedPose.isEmpty()) {
        lastPose = null;
    }

    return estimatedPose;
}

//TODO
private Optional<EstimatedRobotPose> multiTagPNPStrategy(PhotonPipelineResult result, LogTable table) {
    // Arrays we need declared up front
    var visCorners = new ArrayList<TargetCorner>();
    var knownVisTags = new ArrayList<AprilTag>();
    var fieldToCams = new ArrayList<Pose3d>();
    var fieldToCamsAlt = new ArrayList<Pose3d>();

    if (result.getTargets().size() < 2) {
        // Run fallback strategy instead
        return update(result, this.multiTagFallbackStrategy, table);
    }

    for (var target : result.getTargets()) {
        visCorners.addAll(target.getDetectedCorners());

        var tagPoseOpt = fieldTags.getTagPose(target.getFiducialId());
        if (tagPoseOpt.isEmpty()) {
            reportFiducialPoseError(target.getFiducialId());
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

    // multi-target solvePNP
    if (hasCalibData) {
        var cameraMatrix = cameraMatrixOpt.get();
        var distCoeffs = distCoeffsOpt.get();
        var pnpResults =
                VisionEstimation.estimateCamPosePNP(cameraMatrix, distCoeffs, visCorners, knownVisTags);
        var best =
                new Pose3d()
                        .plus(pnpResults.best) // field-to-camera
                        .plus(robotToCamera.inverse()); // field-to-robot
        // var alt = new Pose3d()
        // .plus(pnpResults.alt) // field-to-camera
        // .plus(robotToCamera.inverse()); // field-to-robot

        return Optional.of(
                new EstimatedRobotPose(best, result.getTimestampSeconds(), result.getTargets()));
    } else {
        // TODO fallback strategy? Should we just always do solvePNP?
        return Optional.empty();
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
private Optional<EstimatedRobotPose> lowestAmbiguityStrategy(PhotonPipelineResult result) {
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
        reportFiducialPoseError(targetFiducialId);
        return Optional.empty();
    }

    return Optional.of(
            new EstimatedRobotPose(
                    targetPosition
                            .get()
                            .transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
                            .transformBy(robotToCamera.inverse()),
                    result.getTimestampSeconds(),
                    result.getTargets()));
}

/**
 * Return the estimated position of the robot using the target with the lowest delta height
 * difference between the estimated and actual height of the camera.
 *
 * @param result pipeline result
 * @return the estimated position of the robot in the FCS and the estimated timestamp of this
 *     estimation.
 */
private Optional<EstimatedRobotPose> closestToCameraHeightStrategy(PhotonPipelineResult result) {
    double smallestHeightDifference = 10e9;
    EstimatedRobotPose closestHeightTarget = null;

    for (PhotonTrackedTarget target : result.targets) {
        int targetFiducialId = target.getFiducialId();

        // Don't report errors for non-fiducial targets. This could also be resolved by
        // adding -1 to
        // the initial HashSet.
        if (targetFiducialId == -1) continue;

        Optional<Pose3d> targetPosition = fieldTags.getTagPose(target.getFiducialId());

        if (targetPosition.isEmpty()) {
            reportFiducialPoseError(target.getFiducialId());
            continue;
        }

        double alternateTransformDelta =
                Math.abs(
                        robotToCamera.getZ()
                                - targetPosition
                                        .get()
                                        .transformBy(target.getAlternateCameraToTarget().inverse())
                                        .getZ());
        double bestTransformDelta =
                Math.abs(
                        robotToCamera.getZ()
                                - targetPosition
                                        .get()
                                        .transformBy(target.getBestCameraToTarget().inverse())
                                        .getZ());

        if (alternateTransformDelta < smallestHeightDifference) {
            smallestHeightDifference = alternateTransformDelta;
            closestHeightTarget =
                    new EstimatedRobotPose(
                            targetPosition
                                    .get()
                                    .transformBy(target.getAlternateCameraToTarget().inverse())
                                    .transformBy(robotToCamera.inverse()),
                            result.getTimestampSeconds(),
                            result.getTargets());
        }

        if (bestTransformDelta < smallestHeightDifference) {
            smallestHeightDifference = bestTransformDelta;
            closestHeightTarget =
                    new EstimatedRobotPose(
                            targetPosition
                                    .get()
                                    .transformBy(target.getBestCameraToTarget().inverse())
                                    .transformBy(robotToCamera.inverse()),
                            result.getTimestampSeconds(),
                            result.getTargets());
        }
    }

    // Need to null check here in case none of the provided targets are fiducial.
    return Optional.ofNullable(closestHeightTarget);
}

/**
 * Return the estimated position of the robot using the target with the lowest delta in the vector
 * magnitude between it and the reference pose.
 *
 * @param result pipeline result
 * @param referencePose reference pose to check vector magnitude difference against.
 * @return the estimated position of the robot in the FCS and the estimated timestamp of this
 *     estimation.
 */
private Optional<EstimatedRobotPose> closestToReferencePoseStrategy(
        PhotonPipelineResult result, Pose3d referencePose) {
    if (referencePose == null) {
        DriverStation.reportError(
                "[PhotonPoseEstimator] Tried to use reference pose strategy without setting the reference!",
                false);
        return Optional.empty();
    }

    double smallestPoseDelta = 10e9;
    EstimatedRobotPose lowestDeltaPose = null;

    for (PhotonTrackedTarget target : result.targets) {
        int targetFiducialId = target.getFiducialId();

        // Don't report errors for non-fiducial targets. This could also be resolved by
        // adding -1 to
        // the initial HashSet.
        if (targetFiducialId == -1) continue;

        Optional<Pose3d> targetPosition = fieldTags.getTagPose(target.getFiducialId());

        if (targetPosition.isEmpty()) {
            reportFiducialPoseError(targetFiducialId);
            continue;
        }

        Pose3d altTransformPosition =
                targetPosition
                        .get()
                        .transformBy(target.getAlternateCameraToTarget().inverse())
                        .transformBy(robotToCamera.inverse());
        Pose3d bestTransformPosition =
                targetPosition
                        .get()
                        .transformBy(target.getBestCameraToTarget().inverse())
                        .transformBy(robotToCamera.inverse());

        double altDifference = Math.abs(calculateDifference(referencePose, altTransformPosition));
        double bestDifference = Math.abs(calculateDifference(referencePose, bestTransformPosition));

        if (altDifference < smallestPoseDelta) {
            smallestPoseDelta = altDifference;
            lowestDeltaPose =
                    new EstimatedRobotPose(
                            altTransformPosition, result.getTimestampSeconds(), result.getTargets());
        }
        if (bestDifference < smallestPoseDelta) {
            smallestPoseDelta = bestDifference;
            lowestDeltaPose =
                    new EstimatedRobotPose(
                            bestTransformPosition, result.getTimestampSeconds(), result.getTargets());
        }
    }
    return Optional.ofNullable(lowestDeltaPose);
}

/**
 * Return the average of the best target poses using ambiguity as weight.
 *
 * @param result pipeline result
 * @return the estimated position of the robot in the FCS and the estimated timestamp of this
 *     estimation.
 */
private Optional<EstimatedRobotPose> averageBestTargetsStrategy(PhotonPipelineResult result) {
    List<Pair<PhotonTrackedTarget, Pose3d>> estimatedRobotPoses = new ArrayList<>();
    double totalAmbiguity = 0;

    for (PhotonTrackedTarget target : result.targets) {
        int targetFiducialId = target.getFiducialId();

        // Don't report errors for non-fiducial targets. This could also be resolved by
        // adding -1 to
        // the initial HashSet.
        if (targetFiducialId == -1) continue;

        Optional<Pose3d> targetPosition = fieldTags.getTagPose(target.getFiducialId());

        if (targetPosition.isEmpty()) {
            reportFiducialPoseError(targetFiducialId);
            continue;
        }

        double targetPoseAmbiguity = target.getPoseAmbiguity();

        // Pose ambiguity is 0, use that pose
        if (targetPoseAmbiguity == 0) {
            return Optional.of(
                    new EstimatedRobotPose(
                            targetPosition
                                    .get()
                                    .transformBy(target.getBestCameraToTarget().inverse())
                                    .transformBy(robotToCamera.inverse()),
                            result.getTimestampSeconds(),
                            result.getTargets()));
        }

        totalAmbiguity += 1.0 / target.getPoseAmbiguity();

        estimatedRobotPoses.add(
                new Pair<>(
                        target,
                        targetPosition
                                .get()
                                .transformBy(target.getBestCameraToTarget().inverse())
                                .transformBy(robotToCamera.inverse())));
    }

    // Take the average

    Translation3d transform = new Translation3d();
    Rotation3d rotation = new Rotation3d();

    if (estimatedRobotPoses.isEmpty()) return Optional.empty();

    for (Pair<PhotonTrackedTarget, Pose3d> pair : estimatedRobotPoses) {
        // Total ambiguity is non-zero confirmed because if it was zero, that pose was
        // returned.
        double weight = (1.0 / pair.getFirst().getPoseAmbiguity()) / totalAmbiguity;
        Pose3d estimatedPose = pair.getSecond();
        transform = transform.plus(estimatedPose.getTranslation().times(weight));
        rotation = rotation.plus(estimatedPose.getRotation().times(weight));
    }

    return Optional.of(
            new EstimatedRobotPose(
                    new Pose3d(transform, rotation), result.getTimestampSeconds(), result.getTargets()));
}

/**
 * Difference is defined as the vector magnitude between the two poses
 *
 * @return The absolute "difference" (>=0) between two Pose3ds.
 */
private double calculateDifference(Pose3d x, Pose3d y) {
    return x.getTranslation().getDistance(y.getTranslation());
}

private void reportFiducialPoseError(int fiducialId) {
    if (!reportedErrors.contains(fiducialId)) {
        DriverStation.reportError(
                "[PhotonPoseEstimator] Tried to get pose of unknown AprilTag: " + fiducialId, false);
        reportedErrors.add(fiducialId);
    }
}

  @Override
  public void toLog(LogTable table) {
    VisionIOInputs.logTransform3d(robotToCamera, table, "robot to camera");
    logPose3d(lastPose, table, "last pose");
    logPose3d(referencePose, table, "reference pose");
  }
  @Override
  public void fromLog(LogTable table) {
    
    
  }
  public abstract void updateInputs();
}
