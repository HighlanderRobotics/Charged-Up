package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.Pair;
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

    /**
     * Runs inverse kinematics for the arm and elevator system
     * @param x coordinate of the target position, in inches in front of the elevator base
     * @param y coordinate of the target position, in inches above the elevator base
     * @return a pair containing the elevator extension in inches and arm angle in radians from the elevator, in that order.
     * Returns an empty optional if theta is not a number.
     */
    public static Optional<Pair<Double, Double>> solveInverseKinematics(double x, double y) {
        double xPrime = (x * Math.cos(-Constants.ElevatorConstants.elevatorAngleRad)) 
            - (x * Math.sin(-Constants.ElevatorConstants.elevatorAngleRad));
        double yPrime = (y * Math.cos(-Constants.ElevatorConstants.elevatorAngleRad)) 
            + (y * Math.sin(-Constants.ElevatorConstants.elevatorAngleRad));
        double theta = -Math.asin(yPrime/Constants.RotatingArmConstants.rotatingArmLengthInches);
        if (theta == Double.NaN) {
            return Optional.empty();
        }
        double r1 = xPrime - (Constants.RotatingArmConstants.rotatingArmLengthInches * Math.cos(theta));
        if (r1 == Double.NaN) {
            return Optional.empty();
        }
        return Optional.of(new Pair<Double,Double>(r1, theta));
    }

    /**
     * Runs inverse kinematics for the arm and elevator system, taking the solution where the arm faces down.
     * @param x coordinate of the target position, in inches in front of the elevator base
     * @param y coordinate of the target position, in inches above the elevator base
     * @return a pair containing the elevator extension in inches and arm angle in radians from the elevator, in that order.
     * Returns an empty optional if theta is not a number.
     */
    public static Optional<Pair<Double, Double>> solveAlternativeInverseKinematics(double x, double y) {
        double xPrime = (x * Math.cos(-Constants.ElevatorConstants.elevatorAngleRad)) 
            - (x * Math.sin(-Constants.ElevatorConstants.elevatorAngleRad));
        double yPrime = (y * Math.cos(-Constants.ElevatorConstants.elevatorAngleRad)) 
            + (y * Math.sin(-Constants.ElevatorConstants.elevatorAngleRad));
        double theta = Math.PI + Math.asin(yPrime/Constants.RotatingArmConstants.rotatingArmLengthInches);
        if (theta == Double.NaN) {
            return Optional.empty();
        }
        double r1 = xPrime - (Constants.RotatingArmConstants.rotatingArmLengthInches * Math.cos(theta));
        if (r1 == Double.NaN) {
            return Optional.empty();
        }
        return Optional.of(new Pair<Double,Double>(r1, theta));
    }

    /** Solves for the "best" solution for inverse kinematics to try and keep the arm level without going past hardstops. */
    public static Optional<Pair<Double, Double>> solveBestInverseKinematics(double x, double y) {
        var s1 = solveInverseKinematics(x, y);
        var s2 = solveAlternativeInverseKinematics(x, y);
        if (s1.isEmpty()) {
            return s2;
        } else if (s2.isPresent()) {
            if (isValid(s1.get())) {
                return s1;
            }
            if (isValid(s2.get())) {
                return s2;
            }
        }

        return Optional.empty();
    }

    public static boolean isValid(Pair<Double, Double> positions) {
        return !(positions.getFirst() < 0 || positions.getFirst() > Constants.ElevatorConstants.maxExtensionInches);
    }
    
    @Override
    public void periodic() {
        if (enabled) {
            useOutput(Constants.ElevatorConstants.PIDController.calculate(getMeasurement()), Constants.ElevatorConstants.PIDController.getSetpoint());
        }
    }
}
