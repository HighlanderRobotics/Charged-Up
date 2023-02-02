package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RotatingArmSubsystem extends SubsystemBase{
    WPI_TalonFX rotatingArmMotor;
    boolean enabled = true;
    public RotatingArmSubsystem () {
        rotatingArmMotor = new WPI_TalonFX(Constants.RotatingArmConstants.rotatingArmMotorID);
    }
    private void useOutput(double output, TrapezoidProfile.State state) {
        rotatingArmMotor.set(ControlMode.PercentOutput, output + Constants.RotatingArmConstants.feedforward.calculate(state.position, state.velocity));
    }
    public void setGoal(double position) {
        Constants.RotatingArmConstants.PIDController.setGoal(position);
    }
    private double getMeasurement() {
        return rotatingArmMotor.getSelectedSensorPosition();
    }
    public static double convertTicksToInches (double ticks) {
        return ticks / 2048 * Constants.RotatingArmConstants.rotatingArmGearRatio;
    }
    public static double convertInchesToTicks (double inches) {
        return inches / Constants.RotatingArmConstants.rotatingArmGearRatio * 2048;
    }
    
    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }

    @Override
    public void periodic () {
        if (enabled) {
            useOutput(Constants.RotatingArmConstants.PIDController.calculate(getMeasurement()), Constants.RotatingArmConstants.PIDController.getSetpoint());
        }
    }
}
