// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.util.Color;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface LEDIO {
  @AutoLog
  public static class LEDIOInputs {
    public int numLeds = 1;
    public Color[] color = new Color[numLeds];

    public LEDIOInputs(int len) {
      numLeds = len;
    }
  }

  public default void updateInputs(LEDIOInputs inputs) {}

  public default void set(int i, Color color) {}

  public default void solid(Color color) {}
}
