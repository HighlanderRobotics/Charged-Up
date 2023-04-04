// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.Grids;

/** Add your docs here. */
public class TapeVisionSubsystem {
    PhotonCamera camera;
    Transform3d cameraPose;

    public TapeVisionSubsystem(String cameraName, Transform3d cameraPose) {
        camera = new PhotonCamera(cameraName);
        this.cameraPose = cameraPose;
    }

    public Pair<List<Pose2d>, Double> getEstimatedPoses(Pose2d previousPose) {
        List<Pose2d> result = new ArrayList<>();
        var cameraResult = camera.getLatestResult();

        // Find pose of each target
        for (PhotonTrackedTarget target : cameraResult.getTargets()) {
            // Figure out which target we're looking at
            // Pose if we're looking at a mid goal
            double distanceMid = (Grids.midConeZ - cameraPose.getZ()) / Math.tan(target.getPitch());
            Translation2d translationMid = previousPose.getTranslation()
                .plus(new Translation2d(distanceMid, previousPose.getRotation().plus(Rotation2d.fromDegrees(target.getYaw()))));
            SmartDashboard.putNumber("tag mid distance", distanceMid);

            // Pose if we're looking at a high goal
            double distanceHigh = (Grids.highConeZ - cameraPose.getZ()) / Math.tan(target.getPitch());
            Translation2d translationHigh = previousPose.getTranslation()
                .plus(new Translation2d(distanceHigh, previousPose.getRotation().plus(Rotation2d.fromDegrees(target.getYaw()))));
            SmartDashboard.putNumber("tag high distance", distanceHigh);
            
            Translation2d bestGoal = null;
            double bestDistance = Double.POSITIVE_INFINITY;

            for (var midGoal : Grids.midTranslations) {
                if (midGoal.getDistance(translationMid) < bestDistance) {
                    bestDistance = midGoal.getDistance(translationMid);
                    bestGoal = midGoal;
                }
            }

            for (var midGoal : Grids.highTranslations) {
                if (midGoal.getDistance(translationHigh) < bestDistance) {
                    bestDistance = midGoal.getDistance(translationHigh);
                    bestGoal = midGoal;
                }
            }

            // Correct pose
            result.add(new Pose2d(bestGoal, previousPose.getRotation()));
        }

        return Pair.of(result, cameraResult.getTimestampSeconds());
    }
}
