// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.SimPhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.CoordinateAxis;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.HRCoordinateSystem;
import frc.lib.math.XMatrix;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Grids;

/** Add your docs here. */
public class TapeVisionSubsystem {
    PhotonCamera camera;
    Transform3d cameraToRobot;

    SimVisionSystem simCamera;
    
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

        // // Find pose of each target
        // for (PhotonTrackedTarget target : cameraResult.getTargets()) {
        //     Translation2d measuredTargetCameraSpace = new Translation2d(target.getYaw(), target.getPitch());
        //     Translation3d bestGoal = null;
        //     double bestDistance = Double.POSITIVE_INFINITY;
        //     Translation2d bestEstimatedCameraSpace = null;
        //     Translation3d goal = Grids.mid3dTranslations[17]
        //     // for (var goal : Grids.mid3dTranslations) {
        //         var estimatedTargetCameraSpace = calculateEstimatedCameraSpace(new Pose3d(goal, new Rotation3d()), new Pose3d(fieldToRobot));
        //         double distance = measuredTargetCameraSpace.getDistance(estimatedTargetCameraSpace);
        //         if (distance < bestDistance) {
        //             bestDistance = distance;
        //             bestGoal = goal;
        //             bestEstimatedCameraSpace = estimatedTargetCameraSpace;
        //         }
        //     // }
        //     SmartDashboard.putNumber("goal pose x", bestGoal.getX());
        //     SmartDashboard.putNumber("goal pose y", bestGoal.getY());
        //     SmartDashboard.putNumber("goal pose z", bestGoal.getZ());

        //     SmartDashboard.putNumber("estimated target cam space yaw", Math.toDegrees(bestEstimatedCameraSpace.getX()));
        //     SmartDashboard.putNumber("estimated target cam space pitch", Math.toDegrees(bestEstimatedCameraSpace.getY()));
        // }

        var estimatedGoalPoseCamSpace = calculateEstimatedCameraSpace(
            new Pose3d(Grids.mid3dTranslations[17], new Rotation3d()), 
            new Pose3d(fieldToRobot));
        SmartDashboard.putNumber("estimated target cam space yaw", Math.toDegrees(estimatedGoalPoseCamSpace.getX()));
        SmartDashboard.putNumber("estimated target cam space pitch", Math.toDegrees(estimatedGoalPoseCamSpace.getY()));

        return result;
    }

    public Translation2d calculateEstimatedCameraSpace (Pose3d fieldPositionTarget, Pose3d robotPose) {
        Transform3d fieldToRobot = new Transform3d(new Pose3d(), new Pose3d(robotPose.getTranslation(), new Rotation3d()));
        SmartDashboard.putNumber("field to robot x", fieldToRobot.getX());
        SmartDashboard.putNumber("field to robot y", fieldToRobot.getY());
        SmartDashboard.putNumber("field to robot z", fieldToRobot.getZ());
        Transform3d fieldToRobotRotated = fieldToRobot.plus(new Transform3d(new Translation3d(), robotPose.getRotation()));
        SmartDashboard.putNumber("field to robot no trans x", fieldToRobotRotated.getX());
        SmartDashboard.putNumber("field to robot no trans y", fieldToRobotRotated.getY());
        SmartDashboard.putNumber("field to robot no trans z", fieldToRobotRotated.getZ());
        Pose3d robotPositionTarget = fieldPositionTarget.transformBy(fieldToRobotRotated.inverse());
        SmartDashboard.putNumber("robot rel pose target x", robotPositionTarget.getX());
        SmartDashboard.putNumber("robot rel pose target y", robotPositionTarget.getY());
        SmartDashboard.putNumber("robot rel pose target z", robotPositionTarget.getZ());
        Translation3d cameraPositionTarget = robotPositionTarget.transformBy(cameraToRobot.inverse()).getTranslation();
        
        return new Translation2d(Math.atan2(cameraPositionTarget.getY(), cameraPositionTarget.getX()), 0.0);
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
        System.out.println("field to robot " + fieldToRobot.toString());

        Matrix<N4, N4> robotToCameraTranslation = XMatrix.translationMatrix(robotToCameraTransfrom.getTranslation());
        Matrix<N4, N4> robotToCameraRotation = XMatrix.zRotationMatrix(-robotToCameraTransfrom.getRotation().getZ());

        Matrix<N4, N4> robotToCamera = robotToCameraTranslation.times(robotToCameraRotation);
        System.out.println("robot to camera " + robotToCamera.toString());

        Matrix<N4, N4> fieldToCamera = fieldToRobot.times(robotToCamera);
        System.out.println("field to camera " + fieldToCamera.toString());

        Vector<N4> tapeFieldSpaceVec = VecBuilder.fill(
            tapeFieldSpace.getX(), 
            tapeFieldSpace.getY(), 
            tapeFieldSpace.getZ(), 
            1.0);

        Vector<N4> tapeRobotTransSpace = XMatrix.mulVector(tapeFieldSpaceVec, fieldToRobotTranslation);
        System.out.println("tape to robot trans " + tapeRobotTransSpace.toString());

        Vector<N4> tapeRobotRotSpace = XMatrix.mulVector(tapeRobotTransSpace, fieldToRobotRotation);
        System.out.println("tape to robot rot " + tapeRobotRotSpace.toString());

        Vector<N4> tapeCameraTransSpace = XMatrix.mulVector(tapeRobotRotSpace, robotToCameraTranslation);
        System.out.println("tape to camera trans " + tapeCameraTransSpace.toString());

        Vector<N4> tapeCameraRotSpace = XMatrix.mulVector(tapeCameraTransSpace, robotToCameraRotation);
        System.out.println("tape to camera rot " + tapeCameraRotSpace.toString());

        return new Translation3d(
            tapeCameraRotSpace.get(0, 0), 
            tapeCameraRotSpace.get(1, 0), 
            tapeCameraRotSpace.get(2, 0));
    }

    /** pair (yaw, pitch) */
    public static Pair<Double, Double> cameraSpaceToAngles(Translation3d cameraSpaceTrans) {
        double pitch = Math.atan2(cameraSpaceTrans.getZ(), cameraSpaceTrans.getX());
        double yaw = -Math.atan2(cameraSpaceTrans.getY(), cameraSpaceTrans.getX());
        return Pair.of(yaw, pitch);
    }

    public static Optional<Translation3d> findClosestTape(double pitch, double yaw) {
        double bestDistance = Double.POSITIVE_INFINITY;
        return Optional.empty();
    }

    public void updateSimCamera(Pose2d pose) {
        simCamera.processFrame(pose);
    }
}
