// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RoutingSubsystem extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("OV5647");
  PhotonPipelineResult result;

  /** Creates a new RoutingSubsystem. */
  public RoutingSubsystem() {}

  public Boolean isFlipped() {
    List<TargetCorner> vertices = null;
    if (!result.hasTargets()) {
      return null;
    }
    try {
      vertices = result.getBestTarget().getDetectedCorners();      
    } catch (NullPointerException e) {
      return null;
    }
    return isFlipped(vertices);
  }

  public Boolean isFlipped(List<TargetCorner> vertices) {
    if (vertices.size() == 0) {
      System.out.print("\nNo vertices\n" + "| || \n|| _|");
      return null;
    }
    double maxSlope = 0;
    int maxSlopeVertex = 0;
    String verticesDashboard = "[";
    for (int i = 0; i < vertices.size(); i++) {
      verticesDashboard += "(" + vertices.get(i).x + ", " + vertices.get(i).y + "), ";
      double slope = Math.abs(getSlope(vertices.get(i), getMidpoint(vertices.get((i + 1) % vertices.size()), vertices.get((i + 2) % vertices.size()))));
      if (slope > maxSlope) {
        maxSlope = slope;
        maxSlopeVertex = i;
      }
    }
    SmartDashboard.putString("Vertices", verticesDashboard + "]");
    SmartDashboard.putNumber("Max slope vertex ", maxSlopeVertex);
    SmartDashboard.putNumber("vertices ", vertices.size());
    return vertices.get(maxSlopeVertex).y > 
      getMidpoint(vertices.get((maxSlopeVertex + 1) % vertices.size()), vertices.get((maxSlopeVertex + 2) % vertices.size())).y;
  }

  private TargetCorner getMidpoint(TargetCorner a, TargetCorner b) {
    return new TargetCorner(Math.abs(a.x - b.x), Math.abs(a.x - b.x));
  }

  private double getSlope(TargetCorner a, TargetCorner b) {
    return (a.y - b.y) / (a.x - b.x);
  }

  public List<TargetCorner> mergePoints(ArrayList<TargetCorner> list) {
  
    while (list.size() > 3) {
      Pair<TargetCorner, TargetCorner> closestCorners = getClosestCorners(list);
      TargetCorner pointA = closestCorners.getFirst();
      list.remove(pointA);
      System.out.println("removed " + pointA.toString());
    }
    SmartDashboard.putNumber("Parsed vertices ", list.size());
    // System.out.print("\nParsed " + result);
    return list;
  }

  private Pair<TargetCorner, TargetCorner> getClosestCorners(ArrayList<TargetCorner> list) {
    Pair<TargetCorner, TargetCorner> result = Pair.of(list.get(0), list.get(1));
    double minDist = Double.MAX_VALUE;
    for (TargetCorner cornerA : list) {
      for (TargetCorner cornerB : list) {
        if (distance(cornerA, cornerB) < minDist && cornerA != cornerB) {
          result = Pair.of(cornerA, cornerB);
          minDist = distance(cornerA, cornerB);

          System.out.println(result.getFirst() + ", " + result.getSecond());
          System.out.println(minDist);
        }
      }
    }
    return result;
  }

  private double distance(TargetCorner a, TargetCorner b) {
    return Math.sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
  }

  public Command updateDashboardCommand() {
    return new InstantCommand(() -> {
      Boolean isFlipped = isFlipped();
      if (isFlipped != null) {
        SmartDashboard.putBoolean("Is cone flipped?", isFlipped.booleanValue());
        SmartDashboard.putBoolean("Cone flipped returns null", false);
      } else {
        SmartDashboard.putBoolean("Cone flipped returns null", true);
      }
    }, this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    result = camera.getLatestResult();
  }
}
