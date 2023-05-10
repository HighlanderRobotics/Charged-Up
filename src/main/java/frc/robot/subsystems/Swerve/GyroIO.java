// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public double headingDegrees;
        public double rollDegrees;
    }

    public default void updateInputs(GyroIOInputs input) {}

    public default Rotation2d getHeading() { return new Rotation2d(); }

    public default double getRollDegrees() { return 0.0; }

    public default void resetHeading(double degrees) {}
}
