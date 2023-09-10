// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;


public interface LEDIO {
  public static class LEDIOInputs implements LoggableInputs {
    public int numLeds = Constants.LEDConstants.ledLength;
    public Color[] colors = new Color[numLeds];

    @Override
    public void toLog(LogTable table) {
      table.put("LED Length", numLeds);
      double[] loggedHexCodes = toColorCodes(colors);
      table.put("LED Colors", loggedHexCodes);
    }

    @Override
    public void fromLog(LogTable table) {
      numLeds = (int) table.getInteger("LED Length", numLeds);
      double[] colorCodes = table.getDoubleArray("LED Colors", toColorCodes(colors));
      for (int i = 0; i < colors.length; i++) {
        colors[i] = new Color(colorCodes[i * 3], colorCodes[(i * 3) + 1], colorCodes[(i * 3) + 2]);
      }
    }
  }

  public abstract void updateInputs(LEDIOInputs inputs);

  public abstract void set(int i, Color color);

  public abstract void solid(Color color);

  private static double[] toColorCodes(Color[] colorCodes) {
    int len = colorCodes.length * 3;
    double[] result = new double[len];
    for (int i = 0; i < colorCodes.length; i++) {
      result[i * 3] = colorCodes[i].red;
      result[(i * 3) + 1] = colorCodes[i].green;
      result[(i * 3) + 2] = colorCodes[i].blue;
    }
    return result;
  }
}
