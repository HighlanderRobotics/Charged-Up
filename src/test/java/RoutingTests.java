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

import java.util.ArrayList;
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
            List.of(new TargetCorner(100, 100), new TargetCorner(200, 100), new TargetCorner(140, 50))
        ));
    }

    @Test
    void upsidedownTest() {
        assertEquals(true, routingSubsystem.isFlipped(
            List.of(new TargetCorner(100, 100), new TargetCorner(200, 100), new TargetCorner(140, 150))
        ));
    }

    @Test
    void skewedTest() {
        assertEquals(true, routingSubsystem.isFlipped(
            List.of(new TargetCorner(100, 100), new TargetCorner(200, 140), new TargetCorner(210, 150))
        ));
    }

    @Test
    void distanceTest0(){
        var list = routingSubsystem.mergePoints(new ArrayList<TargetCorner>(List.of(
            new TargetCorner(100, 100), 
            new TargetCorner(200, 100), 
            new TargetCorner(210, 150), 
            new TargetCorner(210, 150))));
            System.out.println(list);
        assertEquals(
            3, list.size());
    }

    @Test
    void distanceTest1(){
        var list = routingSubsystem.mergePoints(new ArrayList<TargetCorner>(List.of(
            new TargetCorner(100, 100), 
            new TargetCorner(200, 100), 
            new TargetCorner(210, 150), 
            new TargetCorner(215, 150),
            new TargetCorner(250, 150),
            new TargetCorner(310, 250),
            new TargetCorner(210, 120))));
            System.out.println(list);
        assertEquals(
            3, list.size());
    }

    
    @Test
    void distanceTest2(){
        var list = routingSubsystem.mergePoints(new ArrayList<TargetCorner>(List.of(
            new TargetCorner(100, 100), 
            new TargetCorner(200, 100), 
            new TargetCorner(210, 150))));
            System.out.println(list);
        assertEquals(
            3, list.size());
    }
}
