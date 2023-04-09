// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



import frc.robot.Constants;
import frc.robot.subsystems.TapeVisionSubsystem;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class TapeVisionTest {
    TapeVisionSubsystem tapeVisionSubsystem = new TapeVisionSubsystem("limelight-left", Constants.leftCameraToRobot);
    static final double margin = Units.inchesToMeters(2.75);

    @BeforeEach
    void setup() {
        assert HAL.initialize(1000, 0);
    }

    @Test
    void tapeToCameraSpaceTest() {
        System.out.println("\n tape to cam space test");
        System.out.println("camera to robot transform " + Constants.leftCameraToRobot.toString());
        var robotPose = new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(0));
        var tapePose = new Translation3d(14.0, 4.0, 0.6);
        var tapeCameraSpace = TapeVisionSubsystem.transformPointToCameraSpace(
            tapePose, 
            robotPose,
            Constants.leftCameraToRobot);
        System.out.println("tape camera space " + tapeCameraSpace.toString());
        var tapeAngles = TapeVisionSubsystem.cameraSpaceToAngles(tapeCameraSpace);
        System.out.println(
            "tape yaw " + Math.toDegrees(tapeAngles.yaw) 
            + " tape pitch " + Math.toDegrees(tapeAngles.pitch));
    }

    @Test
    void straightOnTest() {
        System.out.println("\nstraight");
        test(180, 20.2, 180);
    }

    @Test
    void offsetRightTest() {
        System.out.println("\noffset right");
        test(180, 80, 180);
    }

    @Test
    void offsetLeftTest() {
        System.out.println("\noffset left");
        test(180, -20, 180);
    }

    @Test
    void rotatedLeftTest() {
        System.out.println("\nrotate left");
        test(180, 20.2, 200);
    }

    @Test
    void rotatedRightTest() {
        System.out.println("\nrotate right");
        test(180, 20.2, 160);
    }

    // @Test 
    // void randomPointsTest() {
    //     for (int i = 0; i < 1000; i++) {
    //         Pose2d randomPose = new Pose2d(
    //             (i / 10 % 10.0) + 2.0, 
    //             i / 100 % 10.0, 
    //             new Rotation2d(((i * 10) / (Math.PI * 2)) % (Math.PI * 2)));
    //         System.out.println("random point " + randomPose.toString());
    //         test(randomPose);
    //     }
    // }

    void test(Pose2d visSimRobotPose) {
        tapeVisionSubsystem.updateSimCamera(visSimRobotPose);
        List<Pose2d> estimatedPoses = new ArrayList<>();
        for (var measurement : tapeVisionSubsystem.getEstimatedPoses(visSimRobotPose)) {
            estimatedPoses.add(measurement.estimatedPose);
        }
        if (estimatedPoses.size() == 0) {
            System.out.println("no targets");
            assertEquals(false, true);
            return;
        }
        Pose2d estimatedPose = estimatedPoses.get(0);
        System.out.println("actual pose " + visSimRobotPose.toString());
        System.out.println("estimated pose " + estimatedPose.toString());
        assertEquals(visSimRobotPose.getX(), estimatedPose.getX(), margin, "Diff X was: " + visSimRobotPose.getTranslation().getDistance(estimatedPose.getTranslation()));
        assertEquals(visSimRobotPose.getY(), estimatedPose.getY(), margin, "Diff Y was: " + visSimRobotPose.getTranslation().getDistance(estimatedPose.getTranslation()));
    }

    void test(double x, double y, double rotDeg) {
        test(new Pose2d(Units.inchesToMeters(x), Units.inchesToMeters(y), Rotation2d.fromDegrees(rotDeg)));
    }

}
