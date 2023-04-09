// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.XMatrix;
import frc.robot.Constants;
import frc.robot.Constants.Grids;

/** Add your docs here. */
public class TapeVisionSubsystem {
    PhotonCamera camera;
    Transform3d cameraToRobot;

    SimVisionSystem simCamera;

    static boolean extensivePrinting = false;
    
    public TapeVisionSubsystem(String cameraName, Transform3d cameraToRobot) {
        camera = new PhotonCamera(cameraName);
        this.cameraToRobot = cameraToRobot;

        simCamera = new SimVisionSystem(
            cameraName, 
            77.6, 
            Constants.leftCameraToRobot.inverse(), 
            4.0, 
            320, 
            240, 
            10);
        
        // for (int i = 0; i < Constants.Grids.mid3dTranslations.length; i++) {
        //     var target = Grids.mid3dTranslations[i];
        //     boolean isCube = i == 1 || i == 4 || i == 7;
        //     if (isCube) continue;
        //     simCamera.addSimVisionTarget(new SimVisionTarget(new Pose3d(target, new Rotation3d()), Units.inchesToMeters(2.0), Units.inchesToMeters(4.0), -1));
        // }
        
        // for (int i = 0; i < Constants.Grids.high3dTranslations.length; i++) {
        //     var target = Grids.high3dTranslations[i];
        //     boolean isCube = i == 1 || i == 4 || i == 7;
        //     if (isCube) continue;
        //     simCamera.addSimVisionTarget(new SimVisionTarget(new Pose3d(target, new Rotation3d()), Units.inchesToMeters(2.0), Units.inchesToMeters(4.0), -1));
        // }
        
        camera.setLED(VisionLEDMode.kOn);
        
        simCamera.addSimVisionTarget(new SimVisionTarget(new Pose3d(Constants.Grids.mid3dTranslations[0], new Rotation3d()), Units.inchesToMeters(2.0), Units.inchesToMeters(4.0), -1));
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

        // Figure out if close enough to run this
        // not 100% sure if rotation culling here is good, remove if stuff breaks
        // if (fieldToRobot.getX() < Constants.Vision.maximumDistanceTapeEst 
        //     || fieldToRobot.getX() > Constants.Grids.fieldWidthX - Constants.Vision.maximumDistanceTapeEst
        //     || fieldToRobot.getRotation().getRadians() > Constants.Vision.maximumAngleTapeEst 
        //         && fieldToRobot.getRotation().getRadians() < Math.PI - Constants.Vision.maximumAngleTapeEst) {
        //     return List.of();
        // }

        List<TapeVisionResult> result = new ArrayList<>();
        var cameraResult = camera.getLatestResult();
        // System.out.println(cameraResult.getTargets().size() + " targets");
        // Find pose of each target
        for (PhotonTrackedTarget target : cameraResult.getTargets()) {
            // Reject targets near edge of fov
            // if (Math.abs(target.getYaw()) > (59.6 - 2.0) / 2.0) continue;
            // find tape we think we are looking at
            var optBestTape = findClosestTape(target.getPitch(), target.getYaw(), fieldToRobot, cameraToRobot);
            // if no tapes fit well, skip this target
            if (optBestTape.isEmpty()) continue;
            var bestTape = optBestTape.get();
            // System.out.println("best tape translation " + bestTape.translation.toString());
            // calculate where robot is based on the target and tape found
            double cameraDistanceToTape = (bestTape.isHigh ? 
                Constants.Grids.highConeZ + cameraToRobot.getZ() 
                : Constants.Grids.midConeZ + cameraToRobot.getZ()) 
                / Math.tan(Math.toRadians(target.getPitch()));
            // System.out.println("camera distance to tape " + cameraDistanceToTape);
            
            Transform2d cameraToTape = new Transform2d(
                new Translation2d(
                    cameraDistanceToTape * Math.cos(Math.toRadians(target.getYaw())), 
                    cameraDistanceToTape * Math.sin(Math.toRadians(target.getYaw()))), 
                Rotation2d.fromDegrees(target.getYaw()));
            // System.out.println("back-calc camera to tape " + cameraToTape.toString());

            Transform2d robotToTape = new Transform2d(
                cameraToRobot.getTranslation().toTranslation2d(), 
                new Rotation2d(cameraToRobot.getRotation().getAngle())).inverse()
                    .plus(cameraToTape).plus(new Transform2d(new Translation2d(), fieldToRobot.getRotation()));
            // System.out.println("back-calc robot to tape " + robotToTape.toString());

            Pose2d correctedPose = new Pose2d(bestTape.translation.toTranslation2d(), new Rotation2d()).transformBy(robotToTape);

            result.add(new TapeVisionResult(
                correctedPose, 
                cameraResult.getLatencyMillis(), 
                bestTape.translation, 
                bestTape.isHigh, 
                fieldToRobot.getTranslation().getDistance(correctedPose.getTranslation())));
        }

        return result;
    }

    public static Translation3d transformPointToCameraSpace(
            Translation3d tapeFieldSpace, 
            Pose2d robotFieldSpace, 
            Transform3d robotToCameraTransfrom) {
        
        Matrix<N4, N4> fieldToRobotTranslation = XMatrix.translationMatrix(
            new Translation3d(-robotFieldSpace.getX(), -robotFieldSpace.getY(), 0)
        );
        Matrix<N4, N4> fieldToRobotRotation = XMatrix.zRotationMatrix(robotFieldSpace.getRotation().getRadians());

        Matrix<N4, N4> fieldToRobot = fieldToRobotTranslation.times(fieldToRobotRotation);
        if (extensivePrinting) System.out.println("field to robot " + fieldToRobot.toString());

        Matrix<N4, N4> robotToCameraTranslation = XMatrix.translationMatrix(robotToCameraTransfrom.getTranslation());
        Matrix<N4, N4> robotToCameraRotation = XMatrix.zRotationMatrix(-robotToCameraTransfrom.getRotation().getZ());

        Matrix<N4, N4> robotToCamera = robotToCameraTranslation.times(robotToCameraRotation);
        if (extensivePrinting) System.out.println("robot to camera " + robotToCamera.toString());

        Matrix<N4, N4> fieldToCamera = fieldToRobot.times(robotToCamera);
        if (extensivePrinting) System.out.println("field to camera " + fieldToCamera.toString());

        Vector<N4> tapeFieldSpaceVec = VecBuilder.fill(
            tapeFieldSpace.getX(), 
            tapeFieldSpace.getY(), 
            tapeFieldSpace.getZ(), 
            1.0);

        Vector<N4> tapeRobotTransSpace = XMatrix.mulVector(tapeFieldSpaceVec, fieldToRobotTranslation);
        if (extensivePrinting) System.out.println("tape to robot trans " + tapeRobotTransSpace.toString());

        Vector<N4> tapeRobotRotSpace = XMatrix.mulVector(tapeRobotTransSpace, fieldToRobotRotation);
        if (extensivePrinting) System.out.println("tape to robot rot " + tapeRobotRotSpace.toString());

        Vector<N4> tapeCameraTransSpace = XMatrix.mulVector(tapeRobotRotSpace, robotToCameraTranslation);
        if (extensivePrinting) System.out.println("tape to camera trans " + tapeCameraTransSpace.toString());

        Vector<N4> tapeCameraRotSpace = XMatrix.mulVector(tapeCameraTransSpace, robotToCameraRotation);
        if (extensivePrinting) System.out.println("tape to camera rot " + tapeCameraRotSpace.toString());

        return new Translation3d(
            tapeCameraRotSpace.get(0, 0), 
            tapeCameraRotSpace.get(1, 0), 
            tapeCameraRotSpace.get(2, 0));
    }

    public static class PitchYawResult {
        public double pitch;
        public double yaw;
        public PitchYawResult (double pitch, double yaw) 
        {this.pitch = pitch; this.yaw = yaw;}
    }

    /** pair (yaw, pitch) */
    public static PitchYawResult cameraSpaceToAngles(Translation3d cameraSpaceTrans) {
        double pitch = Math.atan2(cameraSpaceTrans.getZ(), cameraSpaceTrans.getX());
        double yaw = -Math.atan2(cameraSpaceTrans.getY(), cameraSpaceTrans.getX());
        return new PitchYawResult(-pitch, yaw);
    }

    public static class TapeSearchResult {
        public Translation3d translation;
        public boolean isHigh;
        public double angleError;
        public TapeSearchResult (
            Translation3d translation,
            boolean isHigh,
            double angleError
        ) {
            this.translation = translation;
            this.isHigh = isHigh;
            this.angleError = angleError;
        }
    }

    public static Optional<TapeSearchResult> findClosestTape(double pitch, double yaw, Pose2d robotPose, Transform3d cameraToRobot) {
        double bestDistance = Double.POSITIVE_INFINITY;
        Translation3d bestTape = null;
        boolean isBestHigh = false;
        boolean isFacingTowardsBlue = 
            robotPose.getRotation().getDegrees() % 360 < 90 
            && robotPose.getRotation().getDegrees() % 360 > -90;
        // System.out.println("target " + pitch + ", " + yaw);
        for (Translation3d tape : Constants.Grids.mid3dTranslations) {
            if (tape.getZ() == Grids.midCubeZ) continue;
            // if (isFacingTowardsBlue && tape.getX() > Grids.fieldWidthX / 2.0) continue;
            // else if (!isFacingTowardsBlue && tape.getX() <  Grids.fieldWidthX / 2.0) continue;

            var pitchYawTape = cameraSpaceToAngles(transformPointToCameraSpace(tape, robotPose, cameraToRobot));

            if (Math.abs(Math.toDegrees(pitchYawTape.yaw)) > 60.0 
                || Math.abs(Math.toDegrees(pitchYawTape.pitch)) > 50.0) continue;
            double deltaPitch = Math.sqrt(Math.pow(Math.toDegrees(pitchYawTape.pitch), 2) + pitch * pitch);
            double deltaYaw = Math.sqrt(Math.pow(Math.toDegrees(pitchYawTape.yaw), 2) + yaw * yaw);
            double distance = Math.sqrt(
               deltaPitch * deltaPitch 
                + deltaYaw * deltaYaw);
            if (distance < bestDistance) {
                bestDistance = distance;
                bestTape = tape;
                isBestHigh = false;
            }
            SmartDashboard.putString("target testing", "testing " + tape + " with distance " + distance + 
                " @ " + Math.toDegrees(pitchYawTape.pitch) + ", " + Math.toDegrees(pitchYawTape.yaw));
        }

        for (Translation3d tape : Constants.Grids.high3dTranslations) {
            if (tape.getZ() == Grids.highCubeZ) continue;
            // if (isFacingTowardsBlue && tape.getX() > Grids.fieldWidthX / 2.0) continue;
            // else if (!isFacingTowardsBlue && tape.getX() <  Grids.fieldWidthX / 2.0) continue;

            var pitchYawTape = cameraSpaceToAngles(transformPointToCameraSpace(tape, robotPose, cameraToRobot));

            if (Math.abs(Math.toDegrees(pitchYawTape.yaw)) > 60.0 
                || Math.abs(Math.toDegrees(pitchYawTape.pitch)) > 50.0) continue;
            double deltaPitch = Math.sqrt(Math.pow(Math.toDegrees(pitchYawTape.pitch), 2) + pitch * pitch);
            double deltaYaw = Math.sqrt(Math.pow(Math.toDegrees(pitchYawTape.yaw), 2) + yaw * yaw);
            double distance = Math.sqrt(
               deltaPitch * deltaPitch 
                + deltaYaw * deltaYaw);
            if (distance < bestDistance) {
                bestDistance = distance;
                bestTape = tape;
                isBestHigh = true;
            }
            SmartDashboard.putString("target testing", "testing " + tape + " with distance " + distance + 
                " @ " + Math.toDegrees(pitchYawTape.pitch) + ", " + Math.toDegrees(pitchYawTape.yaw));
        }

        // if (bestDistance > Constants.Vision.maxAllowableDistanceTapeEst) {
        //     return Optional.empty();
        // }

        return Optional.of(new TapeSearchResult(bestTape, isBestHigh, bestDistance));
    }

    public void updateSimCamera(Pose2d pose) {
        simCamera.processFrame(pose);
    }
}
