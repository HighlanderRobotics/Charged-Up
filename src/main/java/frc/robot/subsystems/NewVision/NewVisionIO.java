// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.NewVision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public interface NewVisionIO {
    public static class NewVisionIOInputs implements LoggableInputs {
        double timestamp = 0.0;
        double timeSinceLastTimestamp = 0.0;
        List<PhotonTrackedTarget> targets = new ArrayList<>();

        public static void logPhotonTrackedTarget(PhotonTrackedTarget target, LogTable table, String name) {
            logTransform3d(target.getBestCameraToTarget(), table, name);
            logTransform3d(target.getAlternateCameraToTarget(), table, name);

            table.put("yaw " + name, target.getYaw());
            table.put("pitch " + name, target.getPitch());
            table.put("area " + name, target.getArea());
            table.put("skew " + name, target.getSkew());
            table.put("fiducial id " + name, target.getFiducialId());
            table.put("pose ambiguity " + name, target.getPoseAmbiguity());
        }

        public static void logTransform3d(Transform3d transform3d, LogTable table, String name) {
            double rotation[] = new double[4];
            rotation[0] = transform3d.getRotation().getQuaternion().getW();
            rotation[1] = transform3d.getRotation().getQuaternion().getX();
            rotation[2] = transform3d.getRotation().getQuaternion().getY();
            rotation[3] = transform3d.getRotation().getQuaternion().getZ();
            table.put("rotation " + name, rotation);

            double translation[] = new double[3];
            translation[0] = transform3d.getTranslation().getX();
            translation[1] = transform3d.getTranslation().getY();
            translation[2] = transform3d.getTranslation().getZ();
            table.put("translation " + name, translation);
        }

        @Override
        public void toLog(LogTable table) {
            table.put("timestamp", timestamp);
            table.put("latency", timeSinceLastTimestamp);
            for (int i = 0; i < targets.size(); i++) {
                logPhotonTrackedTarget(targets.get(i), table, String.valueOf(i));
            }
        }
        @Override
        public void fromLog(LogTable table) {
            timestamp = table.getDouble("timestamp", timestamp);
            timeSinceLastTimestamp = table.getDouble("latency", timeSinceLastTimestamp);
        }
    }
    public abstract void updateInputs(NewVisionIOInputs inputs, Pose3d robotPose);
}
