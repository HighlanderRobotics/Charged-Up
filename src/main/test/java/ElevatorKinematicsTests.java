// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package test;

import edu.wpi.first.hal.HAL;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Optional;

import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

/** Add your docs here. */
public class ElevatorKinematicsTests {
    @BeforeEach // this method will run before each test
    void setup() {
        assert HAL.initialize(500, 0); 
    }

    @Test
    void positionOne() {
        assertEquals(Optional.of(new Pair<Double, Double>(0, Math.toRadians(44))), ElevatorSubsystem.solveInverseKinematics(12.5, 0));
    }

    @Test
    void positionTwo() {
        assertEquals(Optional.empty(), ElevatorSubsystem.solveInverseKinematics(20.0, 0));
    }

    @Test
    void constantsAreValid() {
        assertEquals(true, ElevatorSubsystem.solveBestInverseKinematics(Constants.ElevatorConstants.l1Translation).isPresent());

        assertEquals(true, ElevatorSubsystem.solveBestInverseKinematics(Constants.ElevatorConstants.l2TranslationCones).isPresent());
        assertEquals(true, ElevatorSubsystem.solveBestInverseKinematics(Constants.ElevatorConstants.l3TranslationCones).isPresent());

        assertEquals(true, ElevatorSubsystem.solveBestInverseKinematics(Constants.ElevatorConstants.l2TranslationCubes).isPresent());
        assertEquals(true, ElevatorSubsystem.solveBestInverseKinematics(Constants.ElevatorConstants.l3TranslationCubes).isPresent());

        assertEquals(true, ElevatorSubsystem.solveBestInverseKinematics(Constants.ElevatorConstants.defaultPosition).isPresent());

    }

}
