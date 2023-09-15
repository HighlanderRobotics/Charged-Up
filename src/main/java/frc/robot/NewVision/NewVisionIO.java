// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.NewVision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
public interface NewVisionIO {
    public static class NewVisionIOInputs implements LoggableInputs {
        double timestamp = 0.0;
        double timeSinceLastTimestamp = 0.0;
        List<PhotonTrackedTarget> targets = new ArrayList<>();


        @Override
        public void toLog(LogTable table) {
            
            
        }
        @Override
        public void fromLog(LogTable table) {
            
        }
    }
    public abstract void updateInputs(NewVisionIOInputs inputs, Pose3d robotPose);
}
