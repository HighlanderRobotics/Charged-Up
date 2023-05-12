package frc.lib.components;

import edu.wpi.first.wpilibj.DigitalInput;

public class ReversibleDigitalInput {

  final boolean reversed;
  final DigitalInput input;

  public ReversibleDigitalInput(int channel, boolean reversed) {
    input = new DigitalInput(channel);
    this.reversed = reversed;
  }

  public boolean get() {
    return input.get() != reversed;
  }
}
