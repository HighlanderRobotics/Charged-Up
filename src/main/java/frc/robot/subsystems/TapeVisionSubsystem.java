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
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
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
        
        for (int i = 0; i < Constants.Grids.mid3dTranslations.length; i++) {
            var target = Grids.mid3dTranslations[i];
            boolean isCube = i == 1 || i == 4 || i == 7;
            if (isCube) continue;
            simCamera.addSimVisionTarget(new SimVisionTarget(new Pose3d(target, new Rotation3d()), Units.inchesToMeters(2.0), Units.inchesToMeters(4.0), -1));
        }
        
        for (int i = 0; i < Constants.Grids.high3dTranslations.length; i++) {
            var target = Grids.high3dTranslations[i];
            boolean isCube = i == 1 || i == 4 || i == 7;
            if (isCube) continue;
            simCamera.addSimVisionTarget(new SimVisionTarget(new Pose3d(target, new Rotation3d()), Units.inchesToMeters(2.0), Units.inchesToMeters(4.0), -1));
        }
        
        camera.setLED(VisionLEDMode.kOn);
        
        // simCamera.addSimVisionTarget(new SimVisionTarget(new Pose3d(Constants.Grids.mid3dTranslations[0], new Rotation3d()), Units.inchesToMeters(2.0), Units.inchesToMeters(4.0), 0));
        // System.out.println(new Pose3d(Constants.Grids.mid3dTranslations[0], new Rotation3d()).toString());
    }
    
    public static class TapeVisionResult {
        public Pose2d estimatedPose;
        public double timestamp;
        public Translation3d targetUsed;
        public boolean isTargetUsedHigh;
        public double error;
        public TapeVisionResult(
            Pose2d estimatedPose,
            double timestamp,
            Translation3d targetUsed,
            boolean isTargetUsedHigh,
            double error
        ) {
            this.estimatedPose = estimatedPose;
            this.timestamp = timestamp;
            this.targetUsed = targetUsed;
            this.isTargetUsedHigh = isTargetUsedHigh;
            this.error = error;
        }
    }

    public List<TapeVisionResult> getEstimatedPoses(Pose2d fieldToRobot) {
        camera.setLED(VisionLEDMode.kOn);

        List<TapeVisionResult> result = new ArrayList<>();
        var cameraResult = camera.getLatestResult();

        // Find pose of each target
        for (PhotonTrackedTarget target : cameraResult.getTargets()) {
            SmartDashboard.putNumber("vision target x", target.getYaw());
            SmartDashboard.putNumber("vision target y", target.getPitch());
            Rotation3d targetRotation = new Rotation3d(0, Math.toRadians(target.getPitch()), Math.toRadians(target.getYaw()));
            // Figure out which target we're looking at
            // Pose if we're looking at a mid goal
            // Doesnt account for camera pitch, so have a level camera
            double distanceMid = (Grids.midConeZ + cameraToRobot.getZ()) / Math.tan(Math.toRadians(target.getPitch()) - cameraToRobot.getRotation().getY()); 

            // Transform2d cameraToTapeMid = new Transform2d(
            //     new Translation2d(
            //         distanceMid * Math.cos(adjustedYaw), 
            //         distanceMid * Math.sin(adjustedYaw)), 
            //     new Rotation2d(adjustedYaw));

            Transform2d cameraToTapeMid = new Transform2d(
                new Translation2d(
                    target.getBestCameraToTarget().getX(), 
                    target.getBestCameraToTarget().getY()), 
                new Rotation2d(target.getBestCameraToTarget().getRotation().getZ()));

            Transform2d robotToTapeMid = new Transform2d(
                cameraToRobot.getTranslation().toTranslation2d(), 
                new Rotation2d(cameraToRobot.getRotation().getZ())).inverse()
                    .plus(cameraToTapeMid).plus(new Transform2d(new Translation2d(), fieldToRobot.getRotation()));

            Pose2d fieldToTapeMid = fieldToRobot.transformBy(robotToTapeMid);
            // System.out.println("field to tape mid " + fieldToTapeMid.toString());

            // Pose if we're looking at a high goal
            // Doesnt account for camera pitch, so have a level camera
            double distanceHigh = (Grids.highConeZ + cameraToRobot.getZ()) / Math.tan(Math.toRadians(target.getPitch()) - cameraToRobot.getRotation().getY()); 

            // Transform2d cameraToTapeHigh = new Transform2d(
            //     new Translation2d(
            //             distanceHigh * Math.cos(Math.toRadians(target.getYaw())), 
            //             distanceHigh * Math.sin(Math.toRadians(target.getYaw()))), 
            //         Rotation2d.fromDegrees(target.getYaw()));
            
            Transform2d cameraToTapeHigh = new Transform2d(
                new Translation2d(
                    target.getBestCameraToTarget().getX(), 
                    target.getBestCameraToTarget().getY()), 
                new Rotation2d(target.getBestCameraToTarget().getRotation().getZ()));

            Transform2d robotToTapeHigh = new Transform2d(
                cameraToRobot.getTranslation().toTranslation2d(), 
                new Rotation2d(cameraToRobot.getRotation().getZ())).inverse()
                    .plus(cameraToTapeHigh).plus(new Transform2d(new Translation2d(), fieldToRobot.getRotation()));

            Pose2d fieldToTapeHigh = fieldToRobot.transformBy(robotToTapeHigh);
            SmartDashboard.putNumber("tape high distance", distanceHigh);
            
            Translation2d bestGoal = null;
            double bestDistance = Double.POSITIVE_INFINITY;
            Translation2d bestPose = null;
            boolean bestGoalIsHigh = false;

            for (var midGoal : Grids.midTranslations) {
                if (midGoal.getDistance(fieldToTapeMid.getTranslation()) < bestDistance) {
                    bestDistance = midGoal.getDistance(fieldToTapeMid.getTranslation());
                    bestGoal = midGoal;
                    bestGoalIsHigh = false;
                    bestPose = bestGoal.plus(robotToTapeMid.getTranslation());
                    // The above doesnt work if the robot isnt pointing straight ahead, this fixes that case
                    double rotationNeeded = fieldToRobot.getRotation().minus(new Rotation2d(Math.PI)).getRadians();
                    bestPose = new Translation2d(
                        (((bestPose.getX() - bestGoal.getX()) * Math.cos(rotationNeeded))) 
                            - (((bestPose.getY() - bestGoal.getY()) * Math.sin(rotationNeeded))) 
                            + bestGoal.getX(),
                        ((bestPose.getX() - bestGoal.getX()) * Math.sin(rotationNeeded))
                            + ((bestPose.getY() - bestGoal.getY()) * Math.cos(rotationNeeded) )
                            + bestGoal.getY());
                }
            }

            for (var highGoal : Grids.highTranslations) {
                if (highGoal.getDistance(fieldToTapeHigh.getTranslation()) < bestDistance) {
                    bestDistance = highGoal.getDistance(fieldToTapeHigh.getTranslation());
                    bestGoal = highGoal;
                    bestGoalIsHigh = true;
                    bestPose = bestGoal.plus(robotToTapeHigh.getTranslation());
                    // The above doesnt work if the robot isnt pointing straight ahead, this fixes that case
                    double rotationNeeded = fieldToRobot.getRotation().minus(new Rotation2d(Math.PI)).getRadians();
                    bestPose = new Translation2d(
                        (((bestPose.getX() - bestGoal.getX()) * Math.cos(rotationNeeded))) 
                            - (((bestPose.getY() - bestGoal.getY()) * Math.sin(rotationNeeded))) 
                            + bestGoal.getX(),
                        ((bestPose.getX() - bestGoal.getX()) * Math.sin(rotationNeeded))
                            + ((bestPose.getY() - bestGoal.getY()) * Math.cos(rotationNeeded) )
                            + bestGoal.getY());
                }
            }

            if (bestPose == null || bestGoal == null) {
                continue;
            }

            SmartDashboard.putNumber("Goal Pose X", bestGoal.getX());
            SmartDashboard.putNumber("Goal Pose Y", bestGoal.getY());
            SmartDashboard.putNumber("distance", bestDistance);
            if (Robot.isSimulation()) {
                System.out.println("best goal " + bestGoal.toString());
                System.out.println("best pose " + bestPose.toString());
                System.out.println("actual pose " + fieldToRobot.getTranslation().toString());
                System.out.println("best distance " + fieldToRobot.getTranslation().getDistance(bestGoal));
                System.out.println("actual distance " + fieldToRobot.getTranslation().getDistance(Constants.Grids.midTranslations[0]));
                System.out.println("camera to tape " + cameraToTapeMid.toString());
                System.out.println("robot to tape " + robotToTapeMid.toString() + "\n");
            }
            // Correct pose
            result.add( new TapeVisionResult(
                new Pose2d(bestPose, fieldToRobot.getRotation()), 
                cameraResult.getTimestampSeconds(), 
                new Translation3d(bestGoal.getX(), bestGoal.getY(), bestGoalIsHigh ? Grids.highConeZ : Grids.midConeZ), 
                bestGoalIsHigh, 
                fieldToRobot.getTranslation().getDistance(bestPose)) );
        }

        return result;
    }

    public void updateSimCamera(Pose2d pose) {
        simCamera.processFrame(pose);
    }
}
