package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.components.HighlanderFalcon;
import frc.lib.components.ReversibleDigitalInput;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    HighlanderFalcon elevatorMotor;
    HighlanderFalcon elevatorFollower;
    boolean enabled = true;
    boolean isZeroing; 
    public ReversibleDigitalInput limitSwitch = new ReversibleDigitalInput(Constants.ElevatorConstants.elevatorLimitSwitchID, true);
    TrapezoidProfile.State lastState = new TrapezoidProfile.State();
    TrapezoidProfile.State goal = new TrapezoidProfile.State();

    public ElevatorSubsystem() {
        elevatorMotor = new HighlanderFalcon(
            Constants.ElevatorConstants.elevatorMotorID, 
            5.45 / 1.0);
        elevatorMotor.config_kP(0, 0.001);
        elevatorMotor.config_kF(0, 4.29e-1);
        elevatorMotor.setInverted(false);
        elevatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice());
        elevatorMotor.configMotionCruiseVelocity(inchesToTicks(Constants.ElevatorConstants.elevatorConstraints.maxVelocity) / 10.0);
        elevatorMotor.configMotionAcceleration(inchesToTicks(Constants.ElevatorConstants.elevatorConstraints.maxAcceleration) / 10.0);
        elevatorMotor.configMotionSCurveStrength(0);
        elevatorMotor.configAllowableClosedloopError(0, 0);
        elevatorFollower = new HighlanderFalcon(Constants.ElevatorConstants.elevatorFollowerID);
        elevatorFollower.set(ControlMode.Follower, Constants.ElevatorConstants.elevatorMotorID);
        elevatorFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30.0, 30.0, 0.5));
        elevatorMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30.0, 30.0, 0.5));
        elevatorMotor.configVoltageCompSaturation(10);
        elevatorFollower.configVoltageCompSaturation(10);
        zeroMotor();
    }

    private void updatePID() {
        double pidOut = Constants.ElevatorConstants.PIDController.calculate(getExtensionInches());
        var setpoint = Constants.ElevatorConstants.PIDController.getSetpoint();
        SmartDashboard.putNumber("elevator setpoint", setpoint.position);
        SmartDashboard.putNumber("elevator setpoint velocity", setpoint.velocity);
        SmartDashboard.putNumber("elevator pid out", pidOut);
        SmartDashboard.putNumber("elevator ff out", Constants.ElevatorConstants.feedforward.calculate(setpoint.velocity));
        elevatorMotor.set(
            ControlMode.PercentOutput, 
            pidOut + Constants.ElevatorConstants.feedforward.calculate(setpoint.velocity));
    }

    public void updateStateSpaceController() {
        lastState = 
            (new TrapezoidProfile(Constants.ElevatorConstants.elevatorConstraints, goal, lastState))
            .calculate(0.020);
        Constants.ElevatorConstants.elevatorLoop.setNextR(lastState.position, lastState.velocity);

        Constants.ElevatorConstants.elevatorLoop.correct(VecBuilder.fill(getExtensionInches()));

        Constants.ElevatorConstants.elevatorLoop.predict(0.020);

        double voltage = Constants.ElevatorConstants.elevatorLoop.getU(0);
        SmartDashboard.putNumber("elevator state space voltage out", voltage);

        elevatorMotor.set(ControlMode.PercentOutput, voltage / 12.0);
    }

    public void updateTalonPID() {
        SmartDashboard.putNumber("Elevator trajectory velocity", ticksToInches(elevatorMotor.getActiveTrajectoryVelocity()) * 10);
        SmartDashboard.putNumber("Elevator FF out", Constants.ElevatorConstants.feedforward.calculate(ticksToInches(elevatorMotor.getActiveTrajectoryVelocity()) * 10));
        SmartDashboard.putNumber("Elevator motor position setpoint", elevatorMotor.getActiveTrajectoryPosition());
        SmartDashboard.putNumber("Elevator position goal native", elevatorMotor.getClosedLoopTarget());
        SmartDashboard.putNumber("Elevator closed loop error", elevatorMotor.getClosedLoopError());
        SmartDashboard.putNumber("Elevator percent out", elevatorMotor.getMotorOutputPercent());
        elevatorMotor.set(
            ControlMode.MotionMagic, 
            inchesToTicks(goal.position), 
            DemandType.ArbitraryFeedForward, 
            Constants.ElevatorConstants.feedforward.calculate(ticksToInches(elevatorMotor.getActiveTrajectoryVelocity()) * 10));
    }

    private void setGoal(double position) {
        Constants.ElevatorConstants.PIDController.reset(getExtensionInches());
        Constants.ElevatorConstants.PIDController.setGoal(position);

        goal = new TrapezoidProfile.State(position, 0);
    }

    private void setGoal(double position, double velocity) {
        Constants.ElevatorConstants.PIDController.reset(getExtensionInches());
        Constants.ElevatorConstants.PIDController.setGoal(new State(position, velocity));

        goal = new TrapezoidProfile.State(position, velocity);
    }

    public double getExtensionInches() {
        return elevatorMotor.getRotations() * Constants.ElevatorConstants.elevatorSpoolCircumference * 2.5;
    }
    
    public CommandBase zeroElevator() {
        return new RunCommand(() -> 
            {
                isZeroing = true ;
                enabled = false ;
            }, this)
        .finallyDo((boolean i) -> 
            {
            isZeroing = false;
            enabled = true; 
            })
            .until(() -> limitSwitch.get())
            .andThen(() -> elevatorMotor.setSelectedSensorPosition(0));
    }
    public void zeroMotor() {
        elevatorMotor.setSelectedSensorPosition(0);
    }

    public boolean isAtGoal() {
        return Math.abs(goal.position - getExtensionInches()) < 3.0;
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }

    public enum ScoringLevels {
        L1(Constants.ScoringLevels.bottomLevel, Constants.ScoringLevels.bottomLevel, 1),
        L2(Constants.ScoringLevels.midConeLevel, Constants.ScoringLevels.midCubeLevel, 2), 
        L3(Constants.ScoringLevels.topConeLevel, Constants.ScoringLevels.topCubeLevel, 3);
        public double extensionInchesCones;
        public double extensionInchesCubes;
        public int level;
        ScoringLevels(double extensionInchesCones, double extensionInchesCubes, int level) {
            this.extensionInchesCones = extensionInchesCones;
            this.extensionInchesCubes = extensionInchesCubes;
            this.level = level;
        }

        public double getConeInches() {
            return this.extensionInchesCones;
        }

        public double getCubeInches() {
            return this.extensionInchesCubes;
        }
    }

    public CommandBase extendToInchesCommand(DoubleSupplier extensionInches) {
        return new InstantCommand(() -> setGoal(extensionInches.getAsDouble()), this)
            .andThen(new WaitUntilCommand(() -> isAtGoal()), new PrintCommand("extended"));
    }

    public CommandBase extendToInchesCommand(double extensionInches) {
        return extendToInchesCommand(() -> extensionInches);
    }

    static double ticksToInches(double ticks) {
        return (ticks / 2048) * Constants.ElevatorConstants.elevatorSpoolCircumference * 2.5;
    }

    static int inchesToTicks(double inches) {
        return (int) ((inches / (Constants.ElevatorConstants.elevatorSpoolCircumference * 2.5)) * 2048);
    }

    @Override
    public void periodic() {
        if (enabled) {
            updateTalonPID();
        }
        else if (isZeroing){
            elevatorMotor.set(ControlMode.PercentOutput, -0.1, DemandType.ArbitraryFeedForward, 0);
            System.out.println("Is zeroing");
        }
        else {
            elevatorMotor.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
        }

        if (limitSwitch.get()) {
            zeroMotor();
        }

        SmartDashboard.putNumber("elevator goal", Constants.ElevatorConstants.PIDController.getGoal().position);
        SmartDashboard.putNumber("elevator pose inches", getExtensionInches());
        // We might have accidentaly tuned elevator pid with this call on, which modifies the state of the pid controller
        // Basically, dont remove this line it's load bearing
        SmartDashboard.putNumber("elevator pid output", Constants.ElevatorConstants.PIDController.calculate(getExtensionInches()));

        SmartDashboard.putNumber("elevator goal", goal.position);
        SmartDashboard.putNumber("elevator pose inches", getExtensionInches());
        // SmartDashboard.putNumber("elevator native position", getMeasurement());
        SmartDashboard.putBoolean("elevator enable", enabled);
        SmartDashboard.putBoolean("elevator limit switch", limitSwitch.get());
        SmartDashboard.putNumber("elevator current out", elevatorMotor.getSupplyCurrent());
        // SmartDashboard.putBoolean("elevator is at goal", isAtGoal());
    }
}
