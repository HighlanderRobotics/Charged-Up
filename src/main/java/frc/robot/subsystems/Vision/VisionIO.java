// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/** At this point needs to be entirely redone */
public interface VisionIO {
  public static class VisionIOInputs implements LoggableInputs {
    double timestamp = 0.0;
    double timeSinceLastTimestamp = 0.0;
    double numTags = 0;
    List<PhotonTrackedTarget> targets = new ArrayList<>();

    public static void logPhotonTrackedTarget(
        PhotonTrackedTarget target, LogTable table, String name) {

      logTransform3d(target.getBestCameraToTarget(), table, name);
      logTransform3d(target.getAlternateCameraToTarget(), table, "alt " + name);
      logCorners(target, table, name);

      table.put("yaw " + name, target.getYaw());
      table.put("pitch " + name, target.getPitch());
      table.put("area " + name, target.getArea());
      table.put("skew " + name, target.getSkew());
      table.put("fiducial id " + name, target.getFiducialId());
      table.put("pose ambiguity " + name, target.getPoseAmbiguity());
    }

    public static void logCorners(PhotonTrackedTarget target, LogTable table, String name) {
      double[] detectedCornersX = new double[4];
      double[] detectedCornersY = new double[4];
      double[] minAreaRectCornersX = new double[4];
      double[] minAreaRectCornersY = new double[4];

      for (int i = 0; i < 4; i++) {
        detectedCornersX[i] = target.getDetectedCorners().get(i).x;
        detectedCornersY[i] = target.getDetectedCorners().get(i).y;
        minAreaRectCornersX[i] = target.getMinAreaRectCorners().get(i).x;
        minAreaRectCornersY[i] = target.getMinAreaRectCorners().get(i).y;
      }
      table.put("detected corners x " + name, detectedCornersX);
      table.put("detected corners y " + name, detectedCornersY);
      table.put("min area rect corners x " + name, minAreaRectCornersX);
      table.put("min area rect corners Y " + name, minAreaRectCornersY);
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

    public Transform3d getLoggedTransform3d(double[] translation, double[] rotation) {
      Transform3d transform3d =
          new Transform3d(
              new Translation3d(translation[0], translation[1], translation[2]),
              new Rotation3d(new Quaternion(rotation[0], rotation[1], rotation[2], rotation[3])));
      return transform3d;
    }

    public void getLoggedPhotonTrackedTarget(
        PhotonTrackedTarget target, LogTable table, String name) {
      double[] translation = table.getDoubleArray("translation " + name, new double[3]);
      double[] rotation = table.getDoubleArray("rotation " + name, new double[4]);
      double[] altTranslation = table.getDoubleArray("translation alt " + name, new double[3]);
      double[] altRotation = table.getDoubleArray("rotation alt " + name, new double[4]);
      double[] detectedCornersX = table.getDoubleArray("detected corners x " + name, new double[4]);
      double[] detectedCornersY = table.getDoubleArray("detected corners y " + name, new double[4]);
      double[] minAreaRectCornersX =
          table.getDoubleArray("min area rect corners x " + name, new double[4]);
      double[] minAreaRectCornersY =
          table.getDoubleArray("min area rect corners y " + name, new double[4]);

      List<TargetCorner> detectedCorners = new ArrayList<>();
      List<TargetCorner> minAreaRectCorners = new ArrayList<>();

      for (int i = 0; i < 4; i++) {
        detectedCorners.add(new TargetCorner(detectedCornersX[i], detectedCornersY[i]));
        minAreaRectCorners.add(new TargetCorner(minAreaRectCornersX[i], minAreaRectCornersY[i]));
      }
      Transform3d pose = getLoggedTransform3d(translation, rotation);
      Transform3d altPose = getLoggedTransform3d(altTranslation, altRotation);
      targets.add(
          new PhotonTrackedTarget(
              table.getDouble("yaw " + name, target.getYaw()),
              table.getDouble("pitch " + name, target.getPitch()),
              table.getDouble("area " + name, target.getArea()),
              table.getDouble("skew " + name, target.getSkew()),
              (int) (table.getInteger("fiducial id " + name, target.getFiducialId())),
              pose,
              altPose,
              table.getDouble("pose ambiguity " + name, target.getPoseAmbiguity()),
              minAreaRectCorners,
              detectedCorners));
    }

    @Override
    public void toLog(LogTable table) {
      table.put("timestamp", timestamp);
      table.put("latency", timeSinceLastTimestamp);
      for (int i = 0; i < targets.size(); i++) {
        logPhotonTrackedTarget(targets.get(i), table, String.valueOf(i));
        numTags += 1;
      }
      table.put("number of tags", targets.size());
    }

    @Override
    public void fromLog(LogTable table) {
      timestamp = table.getDouble("timestamp", timestamp);
      timeSinceLastTimestamp = table.getDouble("latency", timeSinceLastTimestamp);
      numTags = table.getDouble("number of tags", targets.size());
      for (int i = 0; i < numTags; i++) {
        getLoggedPhotonTrackedTarget(targets.get(i), table, String.valueOf(i));
      }
    }
  }

  public abstract void updateInputs(VisionIOInputs inputs, Pose3d robotPose);
}
