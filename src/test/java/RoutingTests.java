// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeEach;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.RoutingSubsystem;

import java.util.List;

/** Add your docs here. */
public class RoutingTests {
    static double delta = 1e-2;
    RoutingSubsystem routingSubsystem;

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);
        routingSubsystem = new RoutingSubsystem();
    }

    @Test
    void uprightTest() {
        assertEquals(false, routingSubsystem.isFlipped(
            List.of(new TargetCorner(10, 10), new TargetCorner(20, 10), new TargetCorner(14, 5))
        ));
    }

    @Test
    void upsidedownTest() {
        assertEquals(true, routingSubsystem.isFlipped(
            List.of(new TargetCorner(10, 10), new TargetCorner(20, 10), new TargetCorner(14, 15))
        ));
    }

    @Test
    void skewedTest() {
        assertEquals(true, routingSubsystem.isFlipped(
            List.of(new TargetCorner(10, 10), new TargetCorner(20, 14), new TargetCorner(21, 15))
        ));
    }
}
