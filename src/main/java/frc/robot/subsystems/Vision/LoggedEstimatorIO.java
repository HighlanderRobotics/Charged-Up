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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.VisionIO.VisionIOInputs;

public interface LoggedEstimatorIO {
public class LoggedEstimator implements LoggableInputs {

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
    double latencyMillis;

    //if there are methods you want to call from photon pose estimator for some reason you would use this
    private PhotonPoseEstimator estimator = new PhotonPoseEstimator(
    fieldTags, primaryStrategy, camera, robotToCamera);

    public LoggedEstimator(
            AprilTagFieldLayout fieldTags,
            PoseStrategy strategy,
            Transform3d robotToCamera) {
        this.fieldTags = fieldTags;
        this.primaryStrategy = strategy;
        this.robotToCamera = robotToCamera;
    }
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

    public static Pose3d getLoggedPose3d(double[] translation, double[] rotation) {
        Pose3d pose3d =
            new Pose3d(
                new Translation3d(translation[0], translation[1], translation[2]),
                new Rotation3d(new Quaternion(rotation[0], rotation[1], rotation[2], rotation[3])));
        return pose3d;
    }
    
    public static Pose3d getLoggedPose3d(LogTable table, String name) {
        double[] translation = table.getDoubleArray("translation " + name, new double[3]);
        double[] rotation = table.getDoubleArray("rotation " + name, new double[4]);
        return getLoggedPose3d(translation, rotation);
    }

    public static void logAprilTag(AprilTag tag, LogTable table, String name) {
        table.put("ID" + name, tag.ID);
        logPose3d(tag.pose, table, "pose" + name);
    }
    
    public static AprilTag getLoggedAprilTag(LogTable table, String name) {
        AprilTag tag = new AprilTag((int) table.getDouble("ID" + name, -1), getLoggedPose3d(table, name));
        return tag;
    }
    
    public static void logEstimatedRobotPose(EstimatedRobotPose pose, LogTable table, String name) {
        logPose3d(pose.estimatedPose, table, "estimated pose " + name);
        table.put("timestamp seconds " + name, pose.timestampSeconds);
        table.put("targets used", pose.targetsUsed.size());
        for (PhotonTrackedTarget target : pose.targetsUsed) {
            VisionIOInputs.logPhotonTrackedTarget(target, table, String.valueOf(pose.targetsUsed.indexOf(target)) + name);
        }
    }

    public static EstimatedRobotPose getLoggedEstimatedRobotPose(LogTable table, String name) {
        List<PhotonTrackedTarget> targets = new ArrayList<>();
        for (int i = 0; i < table.getDouble("targets used", i); i++) {
            targets.add(VisionIOInputs.getLoggedPhotonTrackedTarget(table, name + i));
        }
        EstimatedRobotPose pose = new EstimatedRobotPose(
            getLoggedPose3d(table, "estimated pose " + name), 
            table.getDouble("timestamp seconds " + name, -1), 
            targets);
        return pose;
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
     * Poll data from the configured cameras and update the estimated position of the robot. Returns
     * empty if there are no cameras set or no targets were found from the cameras.
     *
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
     *     the estimate
     */
    public Optional<EstimatedRobotPose> update(LogTable table) {
        latencyMillis = table.getDouble("latency", latencyMillis);
        List<PhotonTrackedTarget> targets = visionIOInputs.targets;
            if (camera == null) {
                DriverStation.reportError("[PhotonPoseEstimator] Missing camera!", false);
                return Optional.empty();
            }

            PhotonPipelineResult cameraResult = new PhotonPipelineResult(latencyMillis, targets);

            return update(cameraResult, table);
    }

    /**
     * Updates the estimated position of the robot. Returns empty if there are no cameras set or no
     * targets were found from the cameras.
    *
    * @param cameraResult The latest pipeline result from the camera
    * @return an EstimatedRobotPose with an estimated pose, and information about the camera(s) and
    *     pipeline results used to create the estimate
    */
    public Optional<EstimatedRobotPose> update(PhotonPipelineResult cameraResult, LogTable table) {
        // Time in the past -- give up, since the following if expects times > 0
        if (cameraResult.getTimestampSeconds() < 0) {
            return Optional.empty();
        }

        // If the pose cache timestamp was set, and the result is from the same timestamp, return an
        // empty result
        if (poseCacheTimestampSeconds > 0
                && Math.abs(poseCacheTimestampSeconds - cameraResult.getTimestampSeconds()) < 1e-6) {
            return Optional.empty();
        }

        // Remember the timestamp of the current result used
        poseCacheTimestampSeconds = cameraResult.getTimestampSeconds();
        table.put("pose cache timestamp seconds", poseCacheTimestampSeconds);

        // If no targets seen, trivial case -- return empty result
        if (!cameraResult.hasTargets()) {
            return Optional.empty();
        }

        return update(cameraResult, this.primaryStrategy, table);
    }

    private Optional<EstimatedRobotPose> update(PhotonPipelineResult cameraResult, PoseStrategy strat, LogTable table) {
        Optional<EstimatedRobotPose> estimatedPose;
        switch (strat) {
            case LOWEST_AMBIGUITY:
                estimatedPose = lowestAmbiguityStrategy(cameraResult, table);
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
            logPose3d(lastPose, table, "last pose");
        }

        return estimatedPose;
    }



    private Optional<EstimatedRobotPose> multiTagPNPStrategy(PhotonPipelineResult result, LogTable table) {
        // Arrays we need declared up front
        var visCorners = new ArrayList<TargetCorner>();
        var knownVisTags = new ArrayList<AprilTag>();
        var fieldToCams = new ArrayList<Pose3d>();
        var fieldToCamsAlt = new ArrayList<Pose3d>();
        double[] visCornersX = new double[4];
        double[] visCornersY = new double[4];

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
        table.put("has calib data?", hasCalibData);

        for (TargetCorner corner : visCorners) {
            visCornersX[visCorners.indexOf(corner)] = corner.x;
            visCornersY[visCorners.indexOf(corner)] = corner.y;
        }
        table.put("visible corners x", visCornersX);
        table.put("visible corners y", visCornersY);

        for (AprilTag tag : knownVisTags) {
            logAprilTag(tag, table, "known vis tag" + String.valueOf(tag));
        }
        for (Pose3d pose : fieldToCams) {
            logPose3d(pose, table, String.valueOf(fieldToCams.indexOf(pose)) + "field to cams");
        }
        for (Pose3d pose : fieldToCamsAlt) {
            logPose3d(pose, table, String.valueOf(fieldToCams.indexOf(pose)) + "field to cams alt");
        }
        //i kind of dont care about logging all this
        // multi-target solvePNP
        if (hasCalibData) {
            var cameraMatrix = cameraMatrixOpt.get();
            var distCoeffs = distCoeffsOpt.get();
            var pnpResults =
                VisionEstimation.estimateCamPosePNP(cameraMatrix, distCoeffs, visCorners, knownVisTags);
            VisionIOInputs.logTransform3d(pnpResults.best, table, "pnp results best");
            table.put("pnp results bestreprojerr", pnpResults.bestReprojErr);
            var best =
                new Pose3d()
                    .plus(pnpResults.best) // field-to-camera
                    .plus(robotToCamera.inverse()); // field-to-robot
            // var alt = new Pose3d()
            // .plus(pnpResults.alt) // field-to-camera
            // .plus(robotToCamera.inverse()); // field-to-robot
            logPose3d(best, table, "best");
            var estimatedRobotPose = new EstimatedRobotPose(best, result.getTimestampSeconds(), result.getTargets());
            logEstimatedRobotPose(estimatedRobotPose, table, "estimated robot pose - multitag pnp");
            table.put("timestamp seconds", result.getTimestampSeconds());
            return Optional.of(
                estimatedRobotPose);
        } else {
            return lowestAmbiguityStrategy(result, table);
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
    private Optional<EstimatedRobotPose> lowestAmbiguityStrategy(PhotonPipelineResult result, LogTable table) {
        PhotonTrackedTarget lowestAmbiguityTarget = null;
        VisionIOInputs.logPhotonTrackedTarget(lowestAmbiguityTarget, table, "lowest ambiguity target");

        double lowestAmbiguityScore = 10;

        for (PhotonTrackedTarget target : result.targets) {
            double targetPoseAmbiguity = target.getPoseAmbiguity();
            table.put("target pose ambiguity", targetPoseAmbiguity);
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
        table.put("target fiducial id", targetFiducialId);

        Optional<Pose3d> targetPosition = fieldTags.getTagPose(targetFiducialId);
        logPose3d(targetPosition.get(), table, "target position");

        if (targetPosition.isEmpty()) {
            reportFiducialPoseError(targetFiducialId);
            return Optional.empty();
        }
        var estimatedRobotPose = new EstimatedRobotPose(
            targetPosition
                .get()
                .transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
                .transformBy(robotToCamera.inverse()),
                    result.getTimestampSeconds(),
                    result.getTargets());
        logEstimatedRobotPose(estimatedRobotPose, table, "estimated robot pose - lowest ambiguity");
        return Optional.of(estimatedRobotPose);
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
    }
    @Override
    public void fromLog(LogTable table) {
        
    }
}
public abstract void updateInputs();
}
