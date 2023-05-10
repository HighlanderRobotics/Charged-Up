// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;

/** Add your docs here. */
public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    double timestamp = 0.0;
    double timeSinceLastTimestamp = 0.0;
    Pose3d lastPose = new Pose3d();
    ArrayList<Integer> tags = new ArrayList<>();
  }

  public static class VisionMeasurement {
    public EstimatedRobotPose estimation;
    public Matrix<N3, N1> confidence;

    public VisionMeasurement(EstimatedRobotPose estimation, Matrix<N3, N1> confidence) {
      this.estimation = estimation;
      this.confidence = confidence;
    }
  }

  public default List<VisionMeasurement> getMeasurement(Pose2d previousPose) {
    return List.of(new VisionMeasurement(
        new EstimatedRobotPose(new Pose3d(), 0, List.of()),
        Constants.PoseEstimator.VISION_MEASUREMENT_STANDARD_DEVIATIONS));
  }
}
