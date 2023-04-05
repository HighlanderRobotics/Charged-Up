// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package test;

import frc.robot.Constants;
import frc.robot.subsystems.TapeVisionSubsystem;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

/** Add your docs here. */
public class TapeVisionTests {
    TapeVisionSubsystem tapeVisionSubsystem = new TapeVisionSubsystem("limelight-left", Constants.leftCameraToRobot);

    @BeforeEach
    void setup() {
        assert HAL.initialize(1000, 0);
    }

    @Test
    void straightOnTest() {
        Pose2d visSimRobotPose = new Pose2d(Units.inchesToMeters(180), 0.513, new Rotation2d(Math.PI));
        field.getObject("vis sim robot pose").setPose(visSimRobotPose);
        tapeVisionSubsystem.updateSimCamera(visSimRobotPose);
        Pose2d estimatedPose = tapeVisionSubsystem.getEstimatedPoses(visSimRobotPose).getFirst().get(0);
        assertEquals(visSimRobotPose, estimatedPose, 0.2, "Diff was: " + visSimRobotPose.getTranslation().getDistance(estimatedPose.getTranslation()));
    }

    @Test
    void no() {
        assertEquals(1, false);
    }
}
