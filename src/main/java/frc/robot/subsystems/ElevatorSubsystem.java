package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    WPI_TalonFX elevatorMotor;
    boolean enabled = true;
    Mechanism2d mech2d = new Mechanism2d(48, 48);
    MechanismRoot2d root2d = mech2d.getRoot("Elevator Root", 0, 8);
    MechanismLigament2d elevatorLig2d = root2d.append(new MechanismLigament2d(
        "Elevator",
        20, 
        Math.toDegrees(Constants.ElevatorConstants.elevatorAngleRad),
        15,
        new Color8Bit(Color.kPurple)));
    MechanismLigament2d armLig2d = elevatorLig2d.append(new MechanismLigament2d(
        "Arm", 
        Constants.RotatingArmConstants.rotatingArmLengthInches,
        90,
        15,
        new Color8Bit(Color.kLavender)));
    public ElevatorSubsystem() {
        elevatorMotor = new WPI_TalonFX(Constants.ElevatorConstants.elevatorMotorID);
        SmartDashboard.putData("elevatorsim", mech2d);
    }
    private void useOutput(double output, TrapezoidProfile.State state) {
        elevatorMotor.set(ControlMode.PercentOutput, output + Constants.ElevatorConstants.feedforward.calculate(state.velocity));
    }
    public void setGoal(double position) {
        Constants.ElevatorConstants.PIDController.setGoal(position);
    }

    public void setGoal(double position, double velocity) {
        Constants.ElevatorConstants.PIDController.setGoal(new State(position, velocity));
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

    public boolean isAtSetpoint() {
        return Constants.ElevatorConstants.PIDController.atGoal();
    }
    public void extendElevator() {

    }
    public void retractElevator() {

    }

    public void updateMech2d(Pair<Double, Double> state) {
        elevatorLig2d.setLength(state.getFirst());
        armLig2d.setAngle(Math.toDegrees(state.getSecond()));
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
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
            - (y * Math.sin(-Constants.ElevatorConstants.elevatorAngleRad));
        double yPrime = (y * Math.cos(-Constants.ElevatorConstants.elevatorAngleRad)) 
            + (x * Math.sin(-Constants.ElevatorConstants.elevatorAngleRad));
        double theta = -Math.asin(yPrime/Constants.RotatingArmConstants.rotatingArmLengthInches);
        if (theta == Double.NaN) {
            return Optional.empty();
        }
        double r1 = xPrime - (Constants.RotatingArmConstants.rotatingArmLengthInches * Math.cos(theta));
        if (r1 == Double.NaN) {
            return Optional.empty();
        }
        return Optional.of(new Pair<Double,Double>(r1, -theta));
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

    /** Solves for the "best" solution for inverse kinematics to try and keep the arm level without going past hardstops.
     * @param x coordinate of the target position, in inches in front of the elevator base
     * @param y coordinate of the target position, in inches above the elevator base
     * @return a pair containing the elevator extension in inches and arm angle in radians from the elevator, in that order.
     * Returns an empty optional if theta is not a number.
     */
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

    public static Optional<Pair<Double, Double>> solveOffsetIK(double x, double y) {
        var rotated = solveBestInverseKinematics(
            x - Constants.ElevatorConstants.elevatorOffset.getX(), 
            y - Constants.ElevatorConstants.elevatorOffset.getY());
        if (rotated.isEmpty()) {
            return Optional.empty();
        }
        return Optional.of(Pair.of(rotated.get().getFirst(), rotated.get().getSecond() + Constants.RotatingArmConstants.armOffset));
    }

    public static boolean isValid(Pair<Double, Double> positions) {
        return !(positions.getFirst() < 0 || positions.getFirst() > Constants.ElevatorConstants.maxExtensionInches);
    }

    /**Generates and follows a motion profile over a line for the elevator and arm */
    public static CommandBase followLineCommand(
            ElevatorSubsystem elevatorSubsystem, 
            ArmSubsystem armSubsystem, 
            double xStart, 
            double yStart, 
            double xEnd, 
            double yEnd, 
            double time) {
        TrapezoidProfile xProfile = new TrapezoidProfile(
            Constants.ElevatorConstants.elevatorArmSystemConstraints, 
            new State(xEnd, 0.0),
            new State(xStart, 0.0));
        TrapezoidProfile yProfile = new TrapezoidProfile(
            Constants.ElevatorConstants.elevatorArmSystemConstraints, 
            new State(yEnd, 0.0),
            new State(yStart, 0.0));

        Timer timer = new Timer();

        return new InstantCommand(() -> {
            timer.reset();
            timer.start();
        }).andThen(new RunCommand( 
            () -> {
                var xState = xProfile.calculate(timer.get());
                var yState = yProfile.calculate(timer.get());
                var setpoint = solveBestInverseKinematics(
                    xState.position, 
                    yState.position);
                if (setpoint.isPresent()) {
                    elevatorSubsystem.setGoal(setpoint.get().getFirst());
                    armSubsystem.setGoal(setpoint.get().getSecond());
                    elevatorSubsystem.updateMech2d(setpoint.get());
                    SmartDashboard.putNumber("Elevator setpoint", setpoint.get().getFirst());
                    SmartDashboard.putNumber("Arm setpoint", setpoint.get().getSecond());
                } else {
                    SmartDashboard.putNumber("Elevator setpoint", -1);
                    SmartDashboard.putNumber("Arm setpoint", -1);
                }
                SmartDashboard.putNumber("t", timer.get());
            },
            elevatorSubsystem, 
            armSubsystem))
            .until(() -> timer.get() > xProfile.totalTime() && timer.get() > yProfile.totalTime());
    }

    public static CommandBase followSplineCommand(
        ElevatorSubsystem elevatorSubsystem,
        ArmSubsystem armSubsystem,
        Spline[] splines
    ) {
        Timer timer = new Timer();
        var sequence = new SequentialCommandGroup();
        for (Spline spline : splines) {
            if (spline == splines[splines.length - 1]) {
                continue;
            }
            sequence.addCommands(new RunCommand(() -> {
                var state = spline.getPoint(timer.get());
                var xState = state.poseMeters.getX();
                var yState = state.poseMeters.getY();
                var setpoint = solveBestInverseKinematics(
                    xState, 
                    yState);
                if (setpoint.isPresent()) {
                    elevatorSubsystem.setGoal(setpoint.get().getFirst());
                    armSubsystem.setGoal(setpoint.get().getSecond());
                    elevatorSubsystem.updateMech2d(setpoint.get());
                    SmartDashboard.putNumber("Elevator setpoint", setpoint.get().getFirst());
                    SmartDashboard.putNumber("Arm setpoint", setpoint.get().getSecond());
                } else {
                    SmartDashboard.putNumber("Elevator setpoint", -1);
                    SmartDashboard.putNumber("Arm setpoint", -1);
                }
                SmartDashboard.putNumber("X target", xState);
                SmartDashboard.putNumber("Y target", yState);
                SmartDashboard.putNumber("t", timer.get());
            }, elevatorSubsystem, armSubsystem));
        }
        return new InstantCommand(() -> {
            timer.reset();
            timer.start();    
        }).andThen(sequence)
        .until(() -> timer.get() > splines.length - 1);
    }

    public static CommandBase followLinearTrajectoryCommand(
        ElevatorSubsystem elevatorSubsystem, 
        ArmSubsystem armSubsystem, 
        List<Pair<Translation2d, Double>> p) {
        var sequence = new SequentialCommandGroup();
        for (int i = 0; i < p.size() - 1; i++) {
            sequence.addCommands(followLineCommand(
                elevatorSubsystem, 
                armSubsystem, 
                p.get(i).getFirst().getX(), 
                p.get(i).getFirst().getY(), 
                p.get(i + 1).getFirst().getX(), 
                p.get(i + 1).getFirst().getY(), 
                p.get(i).getSecond()));
            System.out.print("Added command " + i);
        }
        return sequence;
    }
    
    @Override
    public void periodic() {
        if (enabled) {
            useOutput(Constants.ElevatorConstants.PIDController.calculate(getMeasurement()), Constants.ElevatorConstants.PIDController.getSetpoint());
        }
    }
}
