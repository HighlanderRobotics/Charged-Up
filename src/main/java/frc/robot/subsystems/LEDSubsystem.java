// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  AddressableLED led;
  AddressableLEDBuffer buffer;
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    led = new AddressableLED(Constants.LEDConstants.ledPort);
    buffer = new AddressableLEDBuffer(Constants.LEDConstants.ledLength);
    led.setLength(buffer.getLength());
    led.start();
  }

  public void setSolid(Color8Bit color) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, color.red, color.green, color.blue);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
