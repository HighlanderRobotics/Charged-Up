// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LEDs.LEDIO.LEDIOInputs;
import java.io.File;
import java.util.function.DoubleSupplier;
import javax.imageio.ImageIO;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {
  LEDIOPWM io = new LEDIOPWM();
  LEDIOInputs inputs = new LEDIOInputs();

  int length = inputs.numLeds;

  int rainbowStart = 0;
  int dashStart = 0;
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {}

  public void setSolid(Color color) {
    io.solid(color);
  }

  public CommandBase setSolidCommand(Color color) {
    return new RunCommand(() -> setSolid(color), this);
  }

  public void setBlinking(Color colorOn, Color colorOff, double secOn, double secOff) {
    for (int i = 0; i < length; i++) {
      if (Timer.getFPGATimestamp() % (secOn + secOff) <= secOn) {
        setSolid(colorOn);
      } else {
        setSolid(colorOff);
      }
    }
  }

  public void setBlinking(Color color, double secOn, double secOff) {
    setBlinking(color, Color.kBlack, secOn, secOff);
  }

  public void setBlinking(Color color, double secOn) {
    setBlinking(color, secOn, secOn);
  }

  public CommandBase setBlinkingCommand(Color color, double secOn) {
    return new RunCommand(() -> setBlinking(color, secOn), this);
  }

  public CommandBase setBlinkingCommand(Color color, DoubleSupplier secOn) {
    return new RunCommand(() -> setBlinking(color, secOn.getAsDouble()), this);
  }

  public CommandBase setBlinkingCommand(Color colorOn, Color colorOff, DoubleSupplier secOn) {
    return new RunCommand(
        () -> setBlinking(colorOn, colorOff, secOn.getAsDouble(), secOn.getAsDouble()), this);
  }

  public void setNoise(Color colorA, Color colorB, int value) {
    try {
      var image = ImageIO.read(new File(Filesystem.getDeployDirectory() + "/ledNoise.png"));
      // LoggingWrapper.shared.add("huh", value);
      // LoggingWrapper.shared.add("test color", (image.getRGB(value, 0) & 0xFF) / 255.0);
      for (int i = 0; i < length; i++) {
        double t = (image.getRGB(value, i) & 0xFF) / 255.0;
        io.set(
            i,
            new Color((int) (colorA.red * t), (int) (colorA.green * t), (int) (colorA.blue * t)));
      }
    } catch (Exception e) {
      setSolid(Color.kRed);
      return;
    }
  }

  public CommandBase setNoiseCommand(Color color) {
    return new RunCommand(
        () -> setNoise(color, Color.kBlack, (int) (Timer.getFPGATimestamp() * 20) % 400), this);
  }

  private static Color dim(Color color, double t) {
    return new Color((int) (color.red * t), (int) (color.green * t), (int) (color.blue * t));
  }

  private static Color interpolate(Color a, Color b, double t) {
    return new Color(
        interpolate(a.red, b.red, t),
        interpolate(a.green, b.green, t),
        interpolate(a.blue, b.blue, t));
  }

  private static int interpolate(double v0, double v1, double t) {
    return (int) (v0 + (t * (v1 - v0)));
  }

  public void setProgress(Color color, double progress) {
    for (int i = 0; i < length; i++) {
      if (i < progress) {
        io.set(i, color);
      } else {
        io.set(i, Color.kBlack);
      }
    }
  }

  private void rainbow() {
    for (int i = 0; i < length; i++) {
      io.set(i, Color.fromHSV(rainbowStart % 180 + i, 255, 255));
    }
    rainbowStart += 6;
  }

  public void runColorAlong(Color colorDash, Color colorBg, int dashLength, int speed) {
    setSolid(colorBg);
    for (int i = dashStart; i < dashStart + dashLength; i++) {
      io.set(i % length, colorDash);
    }

    for (int i = dashStart; i < dashStart + dashLength; i++) {
      io.set(((i % length) + 70) % length, colorDash);
    }

    dashStart += speed;
    dashStart %= length / 2;
  }

  public CommandBase setRainbowCommand() {
    return new RunCommand(() -> rainbow(), this);
  }

  // public CommandBase setBatteryStatusCommand() {
  //   return new RunCommand(() -> {
  //     setProgress(
  //       interpolate(new Color(255, 0, 0), new Color(0, 255, 0),
  //       MathUtil.clamp((RoboRioDataJNI.getVInVoltage() - 10) / 3, 0, 4)),
  //       4);
  //   }, this);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("LEDs", inputs);
  }

  @Override
  public void simulationPeriodic() {}
}
