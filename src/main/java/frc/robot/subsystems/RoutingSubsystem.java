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
    ArrayList<TargetCorner> vertices = null;
    if (!result.hasTargets()) {
      return null;
    }
    try {
      vertices = new ArrayList<TargetCorner>(result.getBestTarget().getDetectedCorners());      
    } catch (NullPointerException e) {
      return null;
    }
    return isFlipped(mergePoints(vertices));
  }

  public Boolean isFlipped(ArrayList<TargetCorner> vertices) {
    if (vertices.size() == 0) {
      System.out.print("\nNo vertices\n" + "| || \n|| _|");
      return null;
    }
    sortCorners(vertices);
    String verticesDashboard = "[";
    for (int i = 0; i < vertices.size(); i++) {
      verticesDashboard += "(" + vertices.get(i).x + ", " + vertices.get(i).y + "), ";
    }
    System.out.println("vertices dashboard " + verticesDashboard);
    SmartDashboard.putString("Vertices", verticesDashboard + "]");
    SmartDashboard.putNumber("vertices ", vertices.size());
    return vertices.get(vertices.size() - 1).y > getMidpoint(vertices.get(0), vertices.get(1)).y;
  }

  private TargetCorner getMidpoint(TargetCorner a, TargetCorner b) {
    return new TargetCorner(Math.abs(a.x - b.x), Math.abs(a.x - b.x));
  }

  private double getSlope(TargetCorner a, TargetCorner b) {
    return (a.y - b.y) / (a.x - b.x);
  }

  public ArrayList<TargetCorner> sortCorners(ArrayList<TargetCorner> list) {
    int n = list.size();
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n - i; j++)
                if (distance(list.get(j), list.get((j + 1) % n)) > distance(list.get((j + 1) % n), list.get((j + 2) % n))) {
                    var temp = list.get(j);
                    list.set(j, list.get((j + 1) % n));
                    list.set((j + 1) % n, temp);
                }
    return list;
  }

  public ArrayList<TargetCorner> mergePoints(ArrayList<TargetCorner> list) {
  
    while (list.size() > 3) {
      Pair<TargetCorner, TargetCorner> closestCorners = getClosestCorners(list);
      TargetCorner pointA = closestCorners.getFirst();
      list.remove(pointA);
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
