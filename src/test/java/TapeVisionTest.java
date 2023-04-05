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
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class TapeVisionTest {
    TapeVisionSubsystem tapeVisionSubsystem = new TapeVisionSubsystem("limelight-left", Constants.leftCameraToRobot);
    static final double margin = Units.inchesToMeters(2);

    @BeforeEach
    void setup() {
        assert HAL.initialize(1000, 0);
    }

    @Test
    void straightOnTest() {
        test(180, 20.2, 180);
    }

    @Test
    void offsetRightTest() {
        test(180, 80, 180);
    }

    @Test
    void offsetLeftTest() {
        test(180, -20, 180);
    }

    @Test
    void rotatedLeftTest() {
        test(180, 20.2, 200);
    }

    @Test
    void rotatedRightTest() {
        test(180, 20.2, 160);
    }

    void test(Pose2d visSimRobotPose) {
        tapeVisionSubsystem.updateSimCamera(visSimRobotPose);
        Pose2d estimatedPose = tapeVisionSubsystem.getEstimatedPoses(visSimRobotPose).getFirst().get(0);
        assertEquals(visSimRobotPose.getX(), estimatedPose.getX(), margin, "Diff X was: " + visSimRobotPose.getTranslation().getDistance(estimatedPose.getTranslation()));
        assertEquals(visSimRobotPose.getY(), estimatedPose.getY(), margin, "Diff Y was: " + visSimRobotPose.getTranslation().getDistance(estimatedPose.getTranslation()));
    }

    void test(double x, double y, double rotDeg) {
        test(new Pose2d(Units.inchesToMeters(x), Units.inchesToMeters(y), Rotation2d.fromDegrees(rotDeg)));
    }

}
