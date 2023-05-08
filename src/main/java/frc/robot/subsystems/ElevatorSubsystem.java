package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.components.HighlanderFalcon;
import frc.lib.components.ReversibleDigitalInput;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class ElevatorSubsystem extends SubsystemBase {
  HighlanderFalcon elevatorMotor;
  HighlanderFalcon elevatorFollower;
  boolean enabled = true;
  boolean isZeroing;
  public ReversibleDigitalInput limitSwitch =
      new ReversibleDigitalInput(Constants.ElevatorConstants.elevatorLimitSwitchID, true);
  TrapezoidProfile.State lastState = new TrapezoidProfile.State();
  TrapezoidProfile.State goal = new TrapezoidProfile.State();

  Mechanism2d mech2d = new Mechanism2d(70, 60);
  MechanismRoot2d root2d = mech2d.getRoot("Elevator Root", 0, 8);
  MechanismLigament2d elevatorLig2d =
      root2d.append(
          new MechanismLigament2d(
              "Elevator",
              20,
              Math.toDegrees(Constants.ElevatorConstants.elevatorAngleRad),
              15,
              new Color8Bit(Color.kPurple)));
  MechanismLigament2d armLig2d =
      elevatorLig2d.append(
          new MechanismLigament2d(
              "Arm",
              Constants.ArmConstants.armLengthInches,
              90,
              15,
              new Color8Bit(Color.kLavender)));

  public ElevatorSubsystem() {
    elevatorMotor = new HighlanderFalcon(Constants.ElevatorConstants.elevatorMotorID, 5.45 / 1.0);
    elevatorMotor.config_kF(0, 0);
    elevatorFollower = new HighlanderFalcon(Constants.ElevatorConstants.elevatorFollowerID);
    elevatorFollower.set(ControlMode.Follower, Constants.ElevatorConstants.elevatorMotorID);
    elevatorFollower.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, 30.0, 30.0, 0.5));
    elevatorMotor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, 30.0, 30.0, 0.5));
    elevatorMotor.configVoltageCompSaturation(10);
    elevatorFollower.configVoltageCompSaturation(10);
    // LoggingWrapper.shared.add("elevatorsim", mech2d);
    zeroMotor();
  }

  private void updatePID() {
    double pidOut = Constants.ElevatorConstants.PIDController.calculate(getExtensionInches());
    var setpoint = Constants.ElevatorConstants.PIDController.getSetpoint();
    // LoggingWrapper.shared.add("elevator setpoint", setpoint.position);
    // LoggingWrapper.shared.add("elevator setpoint velocity", setpoint.velocity);
    // LoggingWrapper.shared.add("elevator pid out", pidOut);
    // LoggingWrapper.shared.add(
    // "elevator ff out", Constants.ElevatorConstants.feedforward.calculate(setpoint.velocity));
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
    // LoggingWrapper.shared.add("elevator state space voltage out", voltage);

    elevatorMotor.set(ControlMode.PercentOutput, voltage / 12.0);
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
    return elevatorMotor.getRotations()
        * Constants.ElevatorConstants.elevatorSpoolCircumference
        * 2.5;
  }

  public CommandBase zeroElevator() {
    return new RunCommand(
            () -> {
              isZeroing = true;
              enabled = false;
            },
            this)
        .finallyDo(
            (boolean i) -> {
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
    return Math.abs(
            Constants.ElevatorConstants.PIDController.getGoal().position - getExtensionInches())
        < 3.0;
  }

  public void updateMech2d(Pair<Double, Double> state) {
    //  elevatorLig2d.setLength(state.getFirst());
    // armLig2d.setAngle(Math.toDegrees(state.getSecond()));
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

  @Override
  public void periodic() {
    if (enabled) {
      updatePID();
    } else if (isZeroing) {
      elevatorMotor.set(ControlMode.PercentOutput, -0.1, DemandType.ArbitraryFeedForward, 0);
      System.out.println("Is zeroing");
    } else {
      elevatorMotor.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
    }

    if (limitSwitch.get()) {
      zeroMotor();
    }

    // LoggingWrapper.shared.add(
    // "elevator goal", Constants.ElevatorConstants.PIDController.getGoal().position);
    // LoggingWrapper.shared.add("elevator pose inches", getExtensionInches());
    // We might have accidentaly tuned elevator pid with this call on, which modifies the state of
    // the pid controller
    // Basically, dont remove this line it's load bearing
    SmartDashboard.putNumber(
        "elevator pid output",
        Constants.ElevatorConstants.PIDController.calculate(getExtensionInches()));

    // LoggingWrapper.shared.add(
    // "elevator goal", Constants.ElevatorConstants.PIDController.getGoal().position);
    // LoggingWrapper.shared.add("elevator pose inches", getExtensionInches());
    // //LoggingWrapper.shared.add("elevator native position", getMeasurement());
    // LoggingWrapper.shared.add("elevator enable", enabled);
    // LoggingWrapper.shared.add("elevator limit switch", limitSwitch.get());
    // //LoggingWrapper.shared.add("elevator is at goal", isAtGoal());
  }
}
