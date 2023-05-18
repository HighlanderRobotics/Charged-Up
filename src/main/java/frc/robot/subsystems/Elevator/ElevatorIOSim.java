// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {

  ElevatorSim physicsSim =
      new ElevatorSim(
          DCMotor.getFalcon500(2),
          Constants.ElevatorConstants.elevatorGearRatio,
          Units.lbsToKilograms(7.5),
          Units.inchesToMeters(1.75),
          0.0,
          58.0,
          false);

  public void updateInputs(ElevatorIOInputs inputs) {
    physicsSim.update(0.020);

    inputs.positionInches = getExtensionInches();
    inputs.percentOut = physicsSim.getOutput().get(0, 0) / 12.0;
    inputs.currentAmps = new double[] {physicsSim.getCurrentDrawAmps(), physicsSim.getCurrentDrawAmps()};
    inputs.switchPressed = getLimitSwitch();
  }

  public void setPercentOut(double percentOut, double ff) {
    Logger.getInstance().recordOutput("Elevator Voltage Out", (percentOut * 12.0) + (ff * 12.0));
    physicsSim.setInputVoltage((percentOut * 12.0) + (ff * 12.0));
  }

  public void setPercentOut(double percentOut) {
    setPercentOut(percentOut, 0.0);
  }

  public void stop() {
    physicsSim.setInputVoltage(0.0);
  }

  public void zeroMotor() {
    // physicsSim.setState(new MatBuilder<N2, N1>(Nat.N2(), Nat.N1()).fill(0.0, 0.0));
  }

  public double getExtensionInches() {
    return Units.metersToInches(physicsSim.getPositionMeters());
  }

  public boolean getLimitSwitch() {
    return false;
  }
}
