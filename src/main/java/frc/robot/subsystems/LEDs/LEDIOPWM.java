// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;


public class LEDIOPWM implements LEDIO {
  AddressableLED led;
  AddressableLEDBuffer buffer;

  public LEDIOPWM() {
    led = new AddressableLED(Constants.LEDConstants.ledPort);
    buffer = new AddressableLEDBuffer(Constants.LEDConstants.ledLength);
    led.setLength(buffer.getLength());
    led.start();
  }

  @Override
  public void updateInputs(LEDIOInputs inputs) {
    for (int i = 0; i < buffer.getLength(); i++) {
      inputs.colors[i] = buffer.getLED(i);
    }
    led.setData(buffer);
  }

  @Override
  public void set(int i, Color color) {
    buffer.setLED(i, color);
  }

  @Override
  public void solid(Color color) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setLED(i, color);
    }
  }
}
