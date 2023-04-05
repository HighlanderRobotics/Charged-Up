// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.SimPhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.Grids;

/** Add your docs here. */
public class TapeVisionSubsystem {
    PhotonCamera camera;
    Transform3d cameraToRobot;

    SimVisionSystem simCamera;

    public TapeVisionSubsystem(String cameraName, Transform3d cameraPose) {
        camera = new PhotonCamera(cameraName);
        this.cameraToRobot = cameraPose;

        simCamera = new SimVisionSystem(
            cameraName, 
            77.6, 
            Constants.leftCameraToRobot.inverse(), 
            4.0, 
            320, 
            240, 
            10);
        
        // for (var target : Constants.Grids.mid3dTranslations) {
        //     simCamera.addSimVisionTarget(new SimVisionTarget(new Pose3d(target, new Rotation3d()), Units.inchesToMeters(2.0), Units.inchesToMeters(4.0), 0));
        // }

        // for (var target : Constants.Grids.high3dTranslations) {
        //     simCamera.addSimVisionTarget(new SimVisionTarget(new Pose3d(target, new Rotation3d()), Units.inchesToMeters(2.0), Units.inchesToMeters(4.0), 0));
        // }

        simCamera.addSimVisionTarget(new SimVisionTarget(new Pose3d(Constants.Grids.mid3dTranslations[0], new Rotation3d()), Units.inchesToMeters(2.0), Units.inchesToMeters(4.0), 0));
    }

    public Pair<List<Pose2d>, Double> getEstimatedPoses(Pose2d fieldToRobot) {
        List<Pose2d> result = new ArrayList<>();
        var cameraResult = camera.getLatestResult();

        // Find pose of each target
        for (PhotonTrackedTarget target : cameraResult.getTargets()) {
            // Figure out which target we're looking at
            // Pose if we're looking at a mid goal
            double distanceMid = (Grids.midConeZ + cameraToRobot.getZ()) / Math.tan(Math.toRadians(target.getPitch())); // Doesnt account for camera pitch, so have a level camera
            Transform2d cameraToTapeMid = new Transform2d(
                new Translation2d(
                    distanceMid * Math.cos(Math.toRadians(target.getYaw())), 
                    distanceMid * Math.sin(Math.toRadians(target.getYaw()))), 
                Rotation2d.fromDegrees(target.getYaw()));
            Transform2d robotToTapeMid = cameraToTapeMid.plus(
                new Transform2d(
                    cameraToRobot.getTranslation().toTranslation2d(), 
                    new Rotation2d(cameraToRobot.getRotation().getAngle())).inverse());
            Pose2d fieldToTapeMid = fieldToRobot.transformBy(robotToTapeMid);
            SmartDashboard.putNumber("tag mid distance", distanceMid);

            // Pose if we're looking at a high goal
            double distanceHigh = (Grids.highConeZ - cameraToRobot.getZ()) / Math.tan(target.getPitch()); // Doesnt account for camera pitch, so have a level camera
            Transform2d cameraToTapeHigh = new Transform2d(
                new Translation2d(
                    distanceMid * Math.cos(Math.toRadians(target.getYaw())), 
                    distanceMid * Math.sin(Math.toRadians(target.getYaw()))), 
                Rotation2d.fromDegrees(target.getYaw()));
            Transform2d robotToTapeHigh = cameraToTapeMid.plus(
                new Transform2d(
                    cameraToRobot.getTranslation().toTranslation2d(), 
                    new Rotation2d(cameraToRobot.getRotation().getAngle())).inverse());
            Pose2d fieldToTapeHigh = fieldToRobot.transformBy(robotToTapeMid);
            SmartDashboard.putNumber("tag high distance", distanceHigh);
            
            Translation2d bestGoal = null;
            double bestDistance = Double.POSITIVE_INFINITY;
            Translation2d bestPose = null;

            for (var midGoal : Grids.midTranslations) {
                if (midGoal.getDistance(fieldToTapeMid.getTranslation()) < bestDistance) {
                    bestDistance = midGoal.getDistance(fieldToTapeMid.getTranslation());
                    bestGoal = midGoal;
                    bestPose = bestGoal.plus(robotToTapeMid.getTranslation());
                }
            }

            for (var highGoal : Grids.highTranslations) {
                if (highGoal.getDistance(fieldToTapeHigh.getTranslation()) < bestDistance) {
                    bestDistance = highGoal.getDistance(fieldToTapeHigh.getTranslation());
                    bestGoal = highGoal;
                    bestPose = bestGoal.plus(robotToTapeHigh.getTranslation());
                }
            }

            if (bestPose == null || bestGoal == null || bestDistance > 4.0) {
                continue;
            }

            SmartDashboard.putNumber("Goal Pose X", bestGoal.getX());
            SmartDashboard.putNumber("Goal Pose Y", bestGoal.getY());

            // Correct pose
            result.add(new Pose2d(bestPose, fieldToRobot.getRotation()));
            break;
        }

        return Pair.of(result, cameraResult.getTimestampSeconds());
    }

    public void updateSimCamera(Pose2d pose) {
        simCamera.processFrame(pose);
    }
}
