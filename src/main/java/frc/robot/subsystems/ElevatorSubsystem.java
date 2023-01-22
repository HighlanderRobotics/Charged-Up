<<<<<<< HEAD
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {}
  public void extend() {}
  public void place() {}
  public void retract() {}
  public boolean isAtSetpoint() { return false; }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
=======
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    WPI_TalonFX elevatorMotor;
    boolean enabled = true;
    public ElevatorFeedforward feedforward = new ElevatorFeedforward(0.0, 0.0, 0.0);
    ProfiledPIDController pidController = new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0,0.0));
    public ElevatorSubsystem() {
        
        elevatorMotor = new WPI_TalonFX(0);
        
    }
    private void useOutput(double output, TrapezoidProfile.State state) {
        elevatorMotor.set(ControlMode.PercentOutput, output + feedforward.calculate(state.velocity));
    }
    public void setGoal(double position) {
        pidController.setGoal(position);
    }
    private double getMeasurement() {
        return elevatorMotor.getSelectedSensorPosition();
    }
    public void extendElevator() {

    }
    public void retractElevator() {

    }
    @Override
    public void periodic() {
        if (enabled) {
            useOutput(pidController.calculate(getMeasurement()), pidController.getSetpoint());
        }
    }
>>>>>>> origin/elevator
}
