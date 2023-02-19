// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.components.HighlanderFalcon;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  HighlanderFalcon intake = new HighlanderFalcon(Constants.MechanismConstants.intakeID);
  DoubleSolenoid solenoid = new DoubleSolenoid(
    PneumaticsModuleType.REVPH, 
    Constants.MechanismConstants.intakeSolenoidForwardID, 
    Constants.MechanismConstants.intakeSolenoidBackwardID);
  // Timer to make sure that the intake has time to extend when we check if its out
  Timer timeSinceExtended = new Timer();
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    timeSinceExtended.start();
  }

  private void run() {
    intake.setPercentOut(0.5);
  }

  private void stop() {
    intake.setPercentOut(0);
  }

  private void extend() {
    solenoid.set(Value.kForward);
    timeSinceExtended.reset();
  }

  private void retract() {
    solenoid.set(Value.kReverse);
  }

  public CommandBase runCommand() {
    return new StartEndCommand(
      () -> {this.run(); this.extend();}, 
      () -> {this.stop(); this.retract();}, 
      this);
  }

  public CommandBase stopCommand() {
    return new RunCommand(
      () -> {this.stop(); this.retract();}, 
      this);
  }

  public CommandBase extendCommand() {
    return new StartEndCommand(() -> this.extend(), () -> this.retract(), this);
  }

  public boolean isExtended() {
    return 
    timeSinceExtended.get() > Constants.MechanismConstants.intakeTimeToExtend 
    && solenoid.get() == Value.kForward;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    SmartDashboard.putBoolean("intake extended?", isExtended());
  }
}
