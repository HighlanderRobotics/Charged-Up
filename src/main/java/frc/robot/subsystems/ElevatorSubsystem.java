package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.components.HighlanderFalcon;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    HighlanderFalcon elevatorMotor;
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
        Constants.ArmConstants.rotatingArmLengthInches,
        90,
        15,
        new Color8Bit(Color.kLavender)));
    private static Level level;

    public ElevatorSubsystem() {
        elevatorMotor = new HighlanderFalcon(Constants.ElevatorConstants.elevatorMotorID);
        SmartDashboard.putData("elevatorsim", mech2d);
    }

    public static enum Level {
        L1,
        L2,
        L3,
        HUMAN_PLAYER
    }

    private void useOutput(double output, TrapezoidProfile.State state) {
        elevatorMotor.set(ControlMode.PercentOutput, output + Constants.ElevatorConstants.feedforward.calculate(state.velocity));
    }

    private void setGoal(double position) {
        Constants.ElevatorConstants.PIDController.setGoal(position);
    }

    private void setGoal(double position, double velocity) {
        Constants.ElevatorConstants.PIDController.setGoal(new State(position, velocity));
    }

    public void setLevel(Level level) {
        this.level = level;
    }
    public Level getLevel() {
        return level;
    }

    private double getMeasurement() {
       return elevatorMotor.getSelectedSensorPosition();
    }

    public double getExtensionInches() {
        return elevatorMotor.getRotations() * Constants.ElevatorConstants.elevatorSpoolCircumference;
    }

    public boolean isAtSetpoint() {
        return Constants.ElevatorConstants.PIDController.atGoal();
    }

    public void updateMech2d(Pair<Double, Double> state) {
        elevatorLig2d.setLength(state.getFirst());
        armLig2d.setAngle(Math.toDegrees(state.getSecond()));
    }

    private void enable() {
        enabled = true;
    }

    private void disable() {
        enabled = false;
    }

    public CommandBase extendToInchesCommand(double extensionInches) {
        return new InstantCommand(() -> setGoal(extensionInches), this).andThen(new RunCommand(() -> {}, this));
    }

    /**
     * Computes the forward kinematics of the arm and elevator system
     * @param theta rotation of the arm in radians ccw positive from the x axis
     * @param r extension of the elevator in inches
     * @return translation2d of the position of the end effector, in inches
     */
    public static Translation2d solveForwardKinematics (double theta, double r) {
        Translation2d pos = new Translation2d(r, Rotation2d.fromRadians(Constants.ElevatorConstants.elevatorAngleRad));
        return pos.plus(new Translation2d(Constants.ArmConstants.rotatingArmLengthInches, theta));
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
        double theta = -Math.asin(yPrime/Constants.ArmConstants.rotatingArmLengthInches);
        if (theta == Double.NaN) {
            return Optional.empty();
        }
        double r1 = xPrime - (Constants.ArmConstants.rotatingArmLengthInches * Math.cos(theta));
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
        double theta = Math.PI + Math.asin(yPrime/Constants.ArmConstants.rotatingArmLengthInches);
        if (theta == Double.NaN) {
            return Optional.empty();
        }
        double r1 = xPrime - (Constants.ArmConstants.rotatingArmLengthInches * Math.cos(theta));
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

    public static Optional<Pair<Double, Double>> solveBestInverseKinematics(Translation2d pose) {
        return solveAlternativeInverseKinematics(pose.getX(), pose.getY());
    }

    public static Optional<Pair<Double, Double>> solveOffsetIK(double x, double y) {
        var rotated = solveBestInverseKinematics(
            x - Constants.ElevatorConstants.elevatorOffset.getX(), 
            y - Constants.ElevatorConstants.elevatorOffset.getY());
        if (rotated.isEmpty()) {
            return Optional.empty();
        }
        return Optional.of(Pair.of(rotated.get().getFirst(), rotated.get().getSecond() + Constants.ArmConstants.armOffset));
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
            double yEnd) {
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
                    // elevatorSubsystem.updateMech2d(setpoint.get());
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
                    // elevatorSubsystem.updateMech2d(setpoint.get());
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
                p.get(i + 1).getFirst().getY()));
            System.out.print("Added command " + i);
        }
        return sequence;
    }

    public static CommandBase goToPoseCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, Translation2d endPose) {
        Translation2d startPos = solveForwardKinematics(elevatorSubsystem.getExtensionInches(), armSubsystem.getRotation().getRadians());
        return followLineCommand(
                elevatorSubsystem, 
                armSubsystem, 
                startPos.getX(), 
                startPos.getY(), 
                endPose.getX(), 
                endPose.getY()); 
    }

    public static CommandBase extendCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, Level level, boolean isCone) {
        Translation2d startPos = solveForwardKinematics(elevatorSubsystem.getExtensionInches(), armSubsystem.getRotation().getRadians());
        if (isCone) {
            return followLineCommand(
                elevatorSubsystem, 
                armSubsystem, 
                startPos.getX(), 
                startPos.getY(), 
                Constants.ElevatorConstants.getGoalTranslationCones(level).getX(), 
                Constants.ElevatorConstants.getGoalTranslationCones(level).getY());
        } else {
            return followLineCommand(
                elevatorSubsystem, 
                armSubsystem, 
                startPos.getX(), 
                startPos.getY(), 
                Constants.ElevatorConstants.getGoalTranslationCubes(level).getX(), 
                Constants.ElevatorConstants.getGoalTranslationCubes(level).getY());
        }

    }
    
    @Override
    public void periodic() {
        if (enabled) {
            useOutput(Constants.ElevatorConstants.PIDController.calculate(getExtensionInches()), Constants.ElevatorConstants.PIDController.getSetpoint());
        }
    }
}
