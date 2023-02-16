// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.components.HighlanderFalcon;
import frc.lib.components.ReversibleDigitalInput;
import frc.robot.Constants;

public class RoutingSubsystem extends SubsystemBase {
  // HighlanderFalcon routingLeft = new HighlanderFalcon(
  //   Constants.MechanismConstants.routingLeftID, 
  //   1.0,
  //   Constants.MechanismConstants.routingKP, 
  //   0, 
  //   0);
  // HighlanderFalcon routingRight = new HighlanderFalcon(
  //   Constants.MechanismConstants.routingRightID, 
  //   1.0,
  //   Constants.MechanismConstants.routingKP, 
  //   0, 
  //   0);
  // HighlanderFalcon routingConveyer = new HighlanderFalcon(
  //   Constants.MechanismConstants.routingConveyerID, 
  //   1.0,
  //   Constants.MechanismConstants.conveyerKP, 
  //   0, 
  //   0);
  ReversibleDigitalInput limitSwitch = new ReversibleDigitalInput(
    Constants.MechanismConstants.routingLimitSwitch, 
    Constants.MechanismConstants.isRoutingSwitchReversed);
  // TODO: talk to routing subteam about logic and stuff
  /** Creates a new RoutingSubsystem. */
  public RoutingSubsystem() {}

  private void run() {
    //routingLeft.setTargetRPM(200); // TODO: find right rpm
    //routingRight.setTargetRPM(200); // TODO: find right rpm
    //routingConveyer.setTargetRPM(200); // TODO: find right rpm
  }

  private void stop() {
    //routingLeft.setTargetRPM(0);
    //routingRight.setTargetRPM(0);
    //routingConveyer.setTargetRPM(0);
  }

  public CommandBase runCommand() { 
    

    return new RunCommand(() -> run()).until(limitSwitch::get);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
