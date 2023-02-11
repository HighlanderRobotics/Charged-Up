package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.components.HighlanderFalcon;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase{
    HighlanderFalcon armMotor;
    boolean enabled = true;
    public ArmSubsystem () {
        armMotor = new HighlanderFalcon(Constants.RotatingArmConstants.rotatingArmMotorID);
    }

    private void useOutput(double output, TrapezoidProfile.State state) {
        armMotor.set(ControlMode.PercentOutput, output + Constants.RotatingArmConstants.feedforward.calculate(state.position, state.velocity));
    }

    public void setGoal(double position) {
        Constants.RotatingArmConstants.PIDController.setGoal(position);
    }

    private double getMeasurement() {
        return armMotor.getSelectedSensorPosition();
    }

    public Rotation2d getRotation() {
        return new Rotation2d(armMotor.getRadians());
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
