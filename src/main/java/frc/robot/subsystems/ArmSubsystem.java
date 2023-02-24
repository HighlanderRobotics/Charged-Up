package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.components.HighlanderFalcon;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase{
    HighlanderFalcon armMotor;
    boolean enabled = true;
    DutyCycleEncoder absEncoder;
    public ArmSubsystem () {
        armMotor = new HighlanderFalcon(Constants.ArmConstants.armMotorID);
        armMotor.setNeutralMode(NeutralMode.Brake);
        armMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
            true, 
            30.0, 
            15.0, 
            0.5));
        absEncoder = new DutyCycleEncoder(Constants.ArmConstants.armEncoderID);
        setGoal(-0.8);
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
        // double clampedPosition = MathUtil.clamp(position, -0.6, -1.5);
        Constants.ArmConstants.PIDController.setGoal(position);
    }

    public CommandBase runToRotationCommand(double radians) {
        return new InstantCommand(() -> setGoal(radians), this).andThen(new WaitUntilCommand(() -> isAtSetpoint()));
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
        double pidOut = Constants.ArmConstants.PIDController.calculate(getRotation().getRadians());
        SmartDashboard.putNumber("arm pidout", pidOut);
        if (enabled) {
            updatePID(pidOut, Constants.ArmConstants.PIDController.getSetpoint());
        }

        SmartDashboard.putNumber("arm radians", getRotation().getRadians());
        SmartDashboard.putNumber("arm encoder native", getMeasurement());
        SmartDashboard.putNumber("arm current", armMotor.getSupplyCurrent());
        SmartDashboard.putNumber("arm goal", Constants.ArmConstants.PIDController.getGoal().position);
    }
}
