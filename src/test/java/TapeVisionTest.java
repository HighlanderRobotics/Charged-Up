// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.VisionIOTape;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;


public class TapeVisionTest {
  VisionIOTape tapeVisionSubsystem =
      new VisionIOTape("limelight-left", Constants.leftCameraToRobot);
  static final double margin = Units.inchesToMeters(2);

  @BeforeEach
  void setup() {
    assert HAL.initialize(1000, 0);
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

  @Test
  void randomPointsTest() {
    for (int i = 0; i < 1000; i++) {
      Pose2d randomPose =
          new Pose2d(
              (i / 10 % 10.0) + 2.0,
              i / 100 % 10.0,
              new Rotation2d(((i * 10) / (Math.PI * 2)) % (Math.PI * 2)));
      System.out.println("random point " + randomPose.toString());
      test(randomPose);
    }
  }

  void test(Pose2d visSimRobotPose) {
    // If we ever get back to this make a sim implementation of VisionIOTape to comply with
    // advantagekit practices
    // tapeVisionSubsystem.updateSimCamera(visSimRobotPose);
    // List<Pose2d> estimatedPoses = tapeVisionSubsystem.getMeasurement(visSimRobotPose).;
    // if (estimatedPoses.size() == 0) {
    //   return;
    // }
    // Pose2d estimatedPose = estimatedPoses.get(0);
    // assertEquals(
    //     visSimRobotPose.getX(),
    //     estimatedPose.getX(),
    //     margin,
    //     "Diff X was: "
    //         + visSimRobotPose.getTranslation().getDistance(estimatedPose.getTranslation()));
    // assertEquals(
    //     visSimRobotPose.getY(),
    //     estimatedPose.getY(),
    //     margin,
    //     "Diff Y was: "
    //         + visSimRobotPose.getTranslation().getDistance(estimatedPose.getTranslation()));
  }

  void test(double x, double y, double rotDeg) {
    test(
        new Pose2d(
            Units.inchesToMeters(x), Units.inchesToMeters(y), Rotation2d.fromDegrees(rotDeg)));
  }
}
