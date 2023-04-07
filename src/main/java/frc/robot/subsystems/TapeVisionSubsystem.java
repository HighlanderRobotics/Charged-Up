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
        
        for (var target : Constants.Grids.mid3dTranslations) {
            simCamera.addSimVisionTarget(new SimVisionTarget(new Pose3d(target, new Rotation3d()), Units.inchesToMeters(2.0), Units.inchesToMeters(4.0), -1));
        }

        for (var target : Constants.Grids.high3dTranslations) {
            simCamera.addSimVisionTarget(new SimVisionTarget(new Pose3d(target, new Rotation3d()), Units.inchesToMeters(2.0), Units.inchesToMeters(4.0), -1));
        }

        // simCamera.addSimVisionTarget(new SimVisionTarget(new Pose3d(Constants.Grids.mid3dTranslations[0], new Rotation3d()), Units.inchesToMeters(2.0), Units.inchesToMeters(4.0), 0));
    }

    public Pair<List<Pose2d>, Double> getEstimatedPoses(Pose2d fieldToRobot) {
        List<Pose2d> result = new ArrayList<>();
        var cameraResult = camera.getLatestResult();

        // Find pose of each target
        for (PhotonTrackedTarget target : cameraResult.getTargets()) {
            // Figure out which target we're looking at
            // Pose if we're looking at a mid goal
            // Doesnt account for camera pitch, so have a level camera
            double distanceMid = (Grids.midConeZ + cameraToRobot.getZ()) / Math.tan(Math.toRadians(target.getPitch())); 

            Transform2d cameraToTapeMid = new Transform2d(
                new Translation2d(
                    distanceMid * Math.cos(Math.toRadians(target.getYaw())), 
                    distanceMid * Math.sin(Math.toRadians(target.getYaw()))), 
                Rotation2d.fromDegrees(target.getYaw()));

            Transform2d robotToTapeMid = new Transform2d(
                cameraToRobot.getTranslation().toTranslation2d(), 
                new Rotation2d(cameraToRobot.getRotation().getAngle())).inverse()
                    .plus(cameraToTapeMid).plus(new Transform2d(new Translation2d(), fieldToRobot.getRotation()));

            Pose2d fieldToTapeMid = fieldToRobot.transformBy(robotToTapeMid);
            System.out.println("field to tape mid " + fieldToTapeMid.toString());

            // Pose if we're looking at a high goal
            // Doesnt account for camera pitch, so have a level camera
            double distanceHigh = (Grids.highConeZ + cameraToRobot.getZ()) / Math.tan(Math.toRadians(target.getPitch())); 

            Transform2d cameraToTapeHigh = new Transform2d(
                new Translation2d(
                        distanceHigh * Math.cos(Math.toRadians(target.getYaw())), 
                        distanceHigh * Math.sin(Math.toRadians(target.getYaw()))), 
                    Rotation2d.fromDegrees(target.getYaw()));

            Transform2d robotToTapeHigh = new Transform2d(
                cameraToRobot.getTranslation().toTranslation2d(), 
                new Rotation2d(cameraToRobot.getRotation().getAngle())).inverse()
                    .plus(cameraToTapeHigh).plus(new Transform2d(new Translation2d(), fieldToRobot.getRotation()));

            Pose2d fieldToTapeHigh = fieldToRobot.transformBy(robotToTapeHigh);
            SmartDashboard.putNumber("tag high distance", distanceHigh);
            
            Translation2d bestGoal = null;
            double bestDistance = Double.POSITIVE_INFINITY;
            Translation2d bestPose = null;

            for (var midGoal : Grids.midTranslations) {
                if (midGoal.getDistance(fieldToTapeMid.getTranslation()) < bestDistance) {
                    bestDistance = midGoal.getDistance(fieldToTapeMid.getTranslation());
                    bestGoal = midGoal;
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

            if (bestPose == null || bestGoal == null || bestDistance > Units.inchesToMeters(5.0)) {
                continue;
            }

            SmartDashboard.putNumber("Goal Pose X", bestGoal.getX());
            SmartDashboard.putNumber("Goal Pose Y", bestGoal.getY());
            if (Robot.isSimulation()) {
                System.out.println("best goal " + bestGoal.toString());
                System.out.println("best pose " + bestPose.toString());
                System.out.println("actual pose " + fieldToRobot.getTranslation().toString());
                System.out.println("robot to tape " + robotToTapeMid.toString());
            }
            // Correct pose
            result.add(new Pose2d(bestPose, fieldToRobot.getRotation()));
        }

        return Pair.of(result, cameraResult.getTimestampSeconds());
    }

    public void updateSimCamera(Pose2d pose) {
        simCamera.processFrame(pose);
    }
}