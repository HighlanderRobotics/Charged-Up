// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.EstimatedRobotPose;

/** At this point needs to be entirely redone */
public interface VisionIO {
  public static class VisionIOInputs implements LoggableInputs {
    double timestamp = 0.0;
    double timeSinceLastTimestamp = 0.0;
    Pose3d lastPose = new Pose3d();
    ArrayList<Long> tags = new ArrayList<>();
    double[] cornersX = new double[] {};
    double[] cornersY = new double[] {};

    @Override
    public void toLog(LogTable table) {
      table.put("Latest Frame Timestamp", timestamp);
      table.put("Latency", timeSinceLastTimestamp);
      table.put(
          "Last Estimated Pose",
          new double[] {
            lastPose.getX(), lastPose.getY(), lastPose.getZ(), lastPose.getRotation().getAngle()
          });
      long[] tagsArray = new long[tags.size()];
      for (int i = 0; i < tagsArray.length; i++) {
        tagsArray[i] = tags.get(i);
      }
      table.put("Tags Used", tagsArray);
      table.put("Target Corners X", cornersX);
      table.put("Target Corners Y", cornersY);
    }

    @Override
    public void fromLog(LogTable table) {
      timestamp = table.getDouble("Latest Frame Timestamp", timestamp);
      timeSinceLastTimestamp = table.getDouble("Latency", timeSinceLastTimestamp);
      var loggedLastPose =
          table.getDoubleArray(
              "Last Estimated Pose",
              new double[] {
                lastPose.getX(), lastPose.getY(), lastPose.getZ(), lastPose.getRotation().getAngle()
              });
      lastPose =
          new Pose3d(
              loggedLastPose[0],
              loggedLastPose[1],
              loggedLastPose[2],
              new Rotation3d(0.0, 0.0, loggedLastPose[3]));
      var loggedTags = table.getIntegerArray("Tags Used", new long[] {});
      for (int i = 0; i < loggedTags.length; i++) {
        tags.set(i, loggedTags[i]);
      }
      cornersX = table.getDoubleArray("Target Corners X", cornersX);
      cornersY = table.getDoubleArray("Target Corners Y", cornersY);
    }
  }

  public static class VisionMeasurement {
    public EstimatedRobotPose estimation;
    public Matrix<N3, N1> confidence;

    public VisionMeasurement(EstimatedRobotPose estimation, Matrix<N3, N1> confidence) {
      this.estimation = estimation;
      this.confidence = confidence;
    }
  }

  public abstract List<VisionMeasurement> getMeasurement(Pose2d previousPose);

  public abstract VisionIOInputs updateInputs(Pose3d robotPose);
}
