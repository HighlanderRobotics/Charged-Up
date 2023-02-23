package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.components.HighlanderFalcon;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase{
    HighlanderFalcon armMotor;
    boolean enabled = false;
    DutyCycleEncoder absEncoder;
    public ArmSubsystem () {
        armMotor = new HighlanderFalcon(Constants.ArmConstants.armMotorID);
        armMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
            true, 
            30.0, 
            15.0, 
            0.5));
        absEncoder = new DutyCycleEncoder(Constants.ArmConstants.armEncoderID);
    }

    private void updatePID(double output, TrapezoidProfile.State state) {
        if (getMeasurement() > Constants.ArmConstants.armMaximumAngle 
            && output > 0) {
            armMotor.setPercentOut(0);
        } else if (getMeasurement() < Constants.ArmConstants.armMinimumAngle
            && output < 0) {
            armMotor.setPercentOut(0);
        } else {
            armMotor.set(ControlMode.PercentOutput, MathUtil.clamp(output, -0.1, 0.1));
        }
    }

    /**0 is down, PI/2 is horizontal */
    public void setGoal(double position) {
        position = MathUtil.clamp(position, Constants.ArmConstants.armMinimumAngle, Constants.ArmConstants.armMaximumAngle);
        Constants.ArmConstants.PIDController.setGoal(position);
    }

    public void setGoal(Rotation2d rotation) {
        setGoal(HighlanderFalcon.rotToNative(rotation.getRotations()) * armMotor.getGearing());
    }

    public CommandBase runToRotationCommand(Rotation2d rotation) {
        return new RunCommand(() -> setGoal(rotation), this);
    }

    public void jogUp() {
        armMotor.setPercentOut(0.1);
    }

    public void jogDown() {
        armMotor.setPercentOut(-0.1);
    }

    public void stop() {
        armMotor.setPercentOut(0);
    }

    private double getMeasurement() {
        return (absEncoder.getAbsolutePosition() - Constants.ArmConstants.armOffset) % 1.0;
    }

    public Rotation2d getRotation() {
        return new Rotation2d((getMeasurement()) * Math.PI * 2).minus(new Rotation2d());
    }

    public boolean isAtSetpoint() {
        return Constants.ArmConstants.PIDController.atGoal();
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
            updatePID(Constants.ArmConstants.PIDController.calculate(getRotation().getRadians()), 
                Constants.ArmConstants.PIDController.getSetpoint());
        }

        SmartDashboard.putNumber("arm radians", getRotation().getRadians());
        SmartDashboard.putNumber("arm encoder native", getMeasurement());
        SmartDashboard.putNumber("arm current", armMotor.getSupplyCurrent());
    }
}
