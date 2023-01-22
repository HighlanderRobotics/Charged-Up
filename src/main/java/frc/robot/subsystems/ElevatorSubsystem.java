package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    WPI_TalonFX elevatorMotor;
    boolean enabled = true;
    public ElevatorSubsystem() {
        elevatorMotor = new WPI_TalonFX(Constants.ElevatorConstants.elevatorMotorID);
        
    }
    private void useOutput(double output, TrapezoidProfile.State state) {
        elevatorMotor.set(ControlMode.PercentOutput, output + Constants.ElevatorConstants.feedforward.calculate(state.velocity));
    }
    public void setGoal(double position) {
        Constants.ElevatorConstants.PIDController.setGoal(position);
    }
    private double getMeasurement() {
        return elevatorMotor.getSelectedSensorPosition();
    }
    public static double convertTicksToInches (double ticks) {
        return ticks / 2048 * Constants.ElevatorConstants.elevatorGearRatio;
    }
    public static double convertInchesToTicks (double inches) {
        return inches / Constants.ElevatorConstants.elevatorGearRatio * 2048;
    }
    public void extendElevator() {

    }
    public void retractElevator() {

    }
    @Override
    public void periodic() {
        if (enabled) {
            useOutput(Constants.ElevatorConstants.PIDController.calculate(getMeasurement()), Constants.ElevatorConstants.PIDController.getSetpoint());
        }
    }
}
