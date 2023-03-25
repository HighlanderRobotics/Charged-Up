// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import javax.imageio.ImageIO;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  AddressableLED led;
  AddressableLEDBuffer buffer;

  int rainbowStart = 0;
  int dashStart = 0;

  // AddressableLEDSim sim;
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    led = new AddressableLED(Constants.LEDConstants.ledPort);
    buffer = new AddressableLEDBuffer(Constants.LEDConstants.ledLength);
    led.setLength(buffer.getLength());
    led.start();

    // sim = new AddressableLEDSim(led);
  }

  public void setSolid(Color8Bit color) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setLED(i, color);
    }
  }

  public CommandBase setSolidCommand(Color8Bit color) {
    return new RunCommand(() -> setSolid(color), this);
  }

  public void setBlinking(Color8Bit colorOn, Color8Bit colorOff, double secOn, double secOff) {
    for (int i = 0; i < buffer.getLength(); i++) {
      if (Timer.getFPGATimestamp() % (secOn + secOff) <= secOn) {
        setSolid(colorOn);
      } else {
        setSolid(colorOff);
      }
    }
  }

  public void setBlinking(Color8Bit color, double secOn, double secOff) {
    setBlinking(color, new Color8Bit(Color.kBlack), secOn, secOff);
  }

  public void setBlinking(Color8Bit color, double secOn) {
    setBlinking(color, secOn, secOn);
  }

  public CommandBase setBlinkingCommand(Color8Bit color, double secOn) {
    return new RunCommand(() -> setBlinking(color, secOn), this);
  }

  public CommandBase setBlinkingCommand(Color8Bit color, DoubleSupplier secOn) {
    return new RunCommand(() -> setBlinking(color, secOn.getAsDouble()), this);
  }

  public CommandBase setBlinkingCommand(Color8Bit colorOn, Color8Bit colorOff, DoubleSupplier secOn) {
    return new RunCommand(() -> setBlinking(colorOn, colorOff, secOn.getAsDouble(), secOn.getAsDouble()), this);
  }

  public void setNoise(Color8Bit colorA, Color8Bit colorB, int value) {
    try {
      var image = ImageIO.read(new File(Filesystem.getDeployDirectory() + "/ledNoise.png"));
      SmartDashboard.putNumber("huh", value);
      SmartDashboard.putNumber("test color", (image.getRGB(value, 0)&0xFF) / 255.0);
      for (int i = 0; i < buffer.getLength(); i++) {
        double t = (image.getRGB(value, i)&0xFF) / 255.0;
        buffer.setRGB(i, (int) (colorA.red * t), (int) (colorA.green * t), (int) (colorA.blue * t));
      }
    } catch (Exception e) {
      setSolid(new Color8Bit(Color.kRed));
      return;
    }
  }

  public CommandBase setNoiseCommand(Color8Bit color) {
    return new RunCommand(() -> setNoise(color, new Color8Bit(Color.kBlack), (int) (Timer.getFPGATimestamp() * 20) % 400), this);
  }

  private static Color8Bit dim(Color8Bit color, double t) {
    return new Color8Bit((int) (color.red * t), (int) (color.green * t), (int) (color.blue * t));
  }

  private static Color8Bit interpolate(Color8Bit a, Color8Bit b, double t) {
    return new Color8Bit(interpolate(a.red, b.red, t), interpolate(a.green, b.green, t), interpolate(a.blue, b.blue, t));
  }

  private static int interpolate(double v0, double v1, double t) {
    return (int) (v0 + (t * (v1 - v0)));
  }

  public void setProgress(Color8Bit color, double progress) {
    for (int i = 0; i < buffer.getLength(); i++) {
      if (i < progress) {
        buffer.setLED(i, color);
      } else {
        buffer.setLED(i, Color.kBlack);
      }
    }
  }

  private void rainbow() {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setHSV(i, rainbowStart % 180 + i, 255, 255);
    }
    rainbowStart += 6;
  }

  public void runColorAlong(Color colorDash, Color8Bit colorBg, int dashLength, int speed) {
    setSolid(colorBg);
    for (int i = dashStart; i < dashStart + dashLength; i++) {
        buffer.setLED(i % buffer.getLength(), colorDash);
    }
    dashStart += speed;
    dashStart %= buffer.getLength();
  }

  public CommandBase setRainbowCommand() {
    return new RunCommand(() -> rainbow(), this);
  }

  // public CommandBase setBatteryStatusCommand() {
  //   return new RunCommand(() -> {
  //     setProgress(
  //       interpolate(new Color8Bit(255, 0, 0), new Color8Bit(0, 255, 0), 
  //       MathUtil.clamp((RoboRioDataJNI.getVInVoltage() - 10) / 3, 0, 4)),
  //       4);
  //   }, this);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    led.setData(buffer);
  } 

  @Override
  public void simulationPeriodic() {
  }
}
