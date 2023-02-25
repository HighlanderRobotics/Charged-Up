package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.components.HighlanderFalcon;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    HighlanderFalcon elevatorMotor;
    HighlanderFalcon elevatorFollower;
    boolean enabled = true;
    Mechanism2d mech2d = new Mechanism2d(70, 60);
    MechanismRoot2d root2d = mech2d.getRoot("Elevator Root", 0, 8);
    MechanismLigament2d elevatorLig2d = root2d.append(new MechanismLigament2d(
        "Elevator",
        20, 
        Math.toDegrees(Constants.ElevatorConstants.elevatorAngleRad),
        15,
        new Color8Bit(Color.kPurple)));
    MechanismLigament2d armLig2d = elevatorLig2d.append(new MechanismLigament2d(
        "Arm", 
        Constants.ArmConstants.armLengthInches,
        90,
        15,
        new Color8Bit(Color.kLavender)));
    public double topLevel = Constants.ScoringLevels.topConeLevel; //to avoid null pointer exception
    public double midLevel = Constants.ScoringLevels.midConeLevel;

    public ElevatorSubsystem() {
        elevatorMotor = new HighlanderFalcon(Constants.ElevatorConstants.elevatorMotorID, 5.45 / 1.0);
        elevatorFollower = new HighlanderFalcon(Constants.ElevatorConstants.elevatorFollowerID);
        elevatorFollower.set(ControlMode.Follower, Constants.ElevatorConstants.elevatorMotorID);
        elevatorFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30.0, 30.0, 0.5));
        elevatorMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30.0, 30.0, 0.5));
        SmartDashboard.putData("elevatorsim", mech2d);
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

    private void setGoal(double position) {
        Constants.ElevatorConstants.PIDController.reset(getExtensionInches());
        Constants.ElevatorConstants.PIDController.setGoal(position);
    }

    private void setGoal(double position, double velocity) {
        Constants.ElevatorConstants.PIDController.reset(getExtensionInches());
        Constants.ElevatorConstants.PIDController.setGoal(new State(position, velocity));
    }

    public void setTopLevel(boolean cone) {
        if (cone = true) {
            topLevel = Constants.ScoringLevels.topConeLevel; //this is the dumbest possible way to do this lol
        } else {
            topLevel = Constants.ScoringLevels.topCubeLevel;
        }
    }
    public double getTopLevel() {
        return topLevel;
    }
    public void setMidLevel(boolean cone) {
        if (cone = true) {
            midLevel = Constants.ScoringLevels.midConeLevel;
        } else {
            midLevel = Constants.ScoringLevels.midCubeLevel;
        }
    }
    public double getMidLevel(){
        return midLevel;
    }

    private double getMeasurement() {
       return elevatorMotor.getSelectedSensorPosition();
    }

    public double getExtensionInches() {
        return elevatorMotor.getRotations() * Constants.ElevatorConstants.elevatorSpoolCircumference * 2.5;
    }

    public void zeroMotor() {
        elevatorMotor.setSelectedSensorPosition(0);
    }

    public boolean isAtSetpoint() {
        return Constants.ElevatorConstants.PIDController.atGoal();
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

    public static enum ScoringLevels {
        L1(Constants.ScoringLevels.bottomLevel, Constants.ScoringLevels.bottomLevel),
        L2(Constants.ScoringLevels.midConeLevel, Constants.ScoringLevels.midCubeLevel), 
        L3(Constants.ScoringLevels.topConeLevel, Constants.ScoringLevels.topCubeLevel);
        public double extensionInchesCones;
        public double extensionInchesCubes;
        ScoringLevels(double extensionInchesCones, double extensionInchesCubes) {
            this.extensionInchesCones = extensionInchesCones;
            this.extensionInchesCubes = extensionInchesCubes;
        }
    }

    public CommandBase extendToInchesCommand(double extensionInches) {
        return new InstantCommand(() -> setGoal(extensionInches), this)
            .andThen(new WaitUntilCommand(() -> isAtSetpoint()));
    }
 
    /**
     * Computes the forward kinematics of the arm and elevator system
     * @param theta rotation of the arm in radians ccw positive from the x axis
     * @param r extension of the elevator in inches
     * @return translation2d of the position of the end effector, in inches
     */
    public static Translation2d solveForwardKinematics (double theta, double r) {
        Translation2d pos = new Translation2d(r, Rotation2d.fromRadians(Constants.ElevatorConstants.elevatorAngleRad));
        return pos.plus(new Translation2d(Constants.ArmConstants.armLengthInches, theta));
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
        double theta = -Math.asin(yPrime/Constants.ArmConstants.armLengthInches);
        if (theta == Double.NaN) {
            return Optional.empty();
        }
        double r1 = xPrime - (Constants.ArmConstants.armLengthInches * Math.cos(theta));
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
        double theta = Math.PI + Math.asin(yPrime/Constants.ArmConstants.armLengthInches);
        if (theta == Double.NaN) {
            return Optional.empty();
        }
        double r1 = xPrime - (Constants.ArmConstants.armLengthInches * Math.cos(theta));
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

    /**Generates and follows a motion profile over a line for the elevator and arm.
     * Suboptimal since it recalculates the motion profile of the elevator and arm each tick, and doesnt really follow a line.
     */
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
        SmartDashboard.putString("target pose", endPose.toString());
        Pair<Double, Double> goal;
        try {
            goal = solveBestInverseKinematics(endPose.getX(), endPose.getY()).orElseThrow();
        } catch (NullPointerException e) {
            return new InstantCommand();
        }
        SmartDashboard.putNumber("goal extension", goal.getFirst());
        SmartDashboard.putNumber("goal rotation", goal.getSecond());
        return new RunCommand(() -> {
            elevatorSubsystem.setGoal(goal.getFirst());
            armSubsystem.setGoal(goal.getSecond());
        }, elevatorSubsystem, armSubsystem);
    }
    
    @Override
    public void periodic() {
        if (enabled) {
            updatePID();
        } else {
            elevatorMotor.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
        }
        SmartDashboard.putNumber("elevator goal", Constants.ElevatorConstants.PIDController.getGoal().position);
        SmartDashboard.putNumber("elevator pose inches", getExtensionInches());
        SmartDashboard.putNumber("elevator native position", getMeasurement());
        // We might have accidentaly tuned elevator pid with this call on, which modifies the state of the pid controller
        // Basically, dont remove this line it's load bearing
        SmartDashboard.putNumber("elevator pid output", Constants.ElevatorConstants.PIDController.calculate(getExtensionInches()));
        SmartDashboard.putBoolean("elevator enable", enabled);
    }
}
