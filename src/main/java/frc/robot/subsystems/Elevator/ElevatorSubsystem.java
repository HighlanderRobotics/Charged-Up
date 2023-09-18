package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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
import frc.robot.Constants;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  ElevatorIO io;
  ElevatorIOInputsAutoLogged inputs;

  boolean enabled = true;
  boolean isZeroing;

  TrapezoidProfile.State lastState = new TrapezoidProfile.State();
  TrapezoidProfile.State goal = new TrapezoidProfile.State();

  Mechanism2d mech2d = new Mechanism2d(Units.inchesToMeters(35), Units.inchesToMeters(60));
  MechanismRoot2d root2d = mech2d.getRoot("Elevator Root", 0, Units.inchesToMeters(0));
  MechanismLigament2d elevatorLig2d =
      root2d.append(
          new MechanismLigament2d(
              "Elevator",
              20,
              Math.toDegrees(Constants.ElevatorConstants.elevatorAngleRad),
              15,
              new Color8Bit(Color.kPurple)));

  public ElevatorSubsystem() {
    io = Robot.isReal() ? new ElevatorIOFalcon() : new ElevatorIOSim();
    inputs = new ElevatorIOInputsAutoLogged();
  }

  private void updatePID() {
    double pidOut = Constants.ElevatorConstants.PIDController.calculate(getExtensionInches());
    var setpoint = Constants.ElevatorConstants.PIDController.getSetpoint();
    io.setPercentOut(pidOut + Constants.ElevatorConstants.feedforward.calculate(setpoint.velocity));
  }

  public void updateStateSpaceController() {
    lastState =
        (new TrapezoidProfile(Constants.ElevatorConstants.elevatorConstraints, goal, lastState))
            .calculate(0.020);
    Constants.ElevatorConstants.elevatorLoop.setNextR(lastState.position, lastState.velocity);

    Constants.ElevatorConstants.elevatorLoop.correct(VecBuilder.fill(getExtensionInches()));

    Constants.ElevatorConstants.elevatorLoop.predict(0.020);

    double voltage = Constants.ElevatorConstants.elevatorLoop.getU(0);

    io.setPercentOut(voltage / 12.0);
  }

  private void setGoal(double position) {
    Constants.ElevatorConstants.PIDController.reset(getExtensionInches());
    Constants.ElevatorConstants.PIDController.setGoal(position);

    goal = new TrapezoidProfile.State(position, 0);
  }

  public double getExtensionInches() {
    return inputs.positionInches;
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
        .until(() -> io.getLimitSwitch())
        .andThen(() -> io.zeroMotor());
  }

  public void zeroMotor() {
    io.zeroMotor();
  }

  public boolean isAtGoal() {
    return Math.abs(
            Constants.ElevatorConstants.PIDController.getGoal().position - getExtensionInches())
        < 3.0;
  }

  public void updateMech2d() {
    elevatorLig2d.setLength(Units.inchesToMeters(getExtensionInches()) * 2.5);
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

  public boolean getLimitSwitch() {
    return io.getLimitSwitch();
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Elevator", inputs);

    // TODO change this to an enum instead of 2 booleans
    if (enabled) {
      updatePID();
      Logger.getInstance().recordOutput("Elevator Mode", "PID");
    } else if (isZeroing) {
      io.setPercentOut(-0.1);
      Logger.getInstance().recordOutput("Elevator Mode", "ZERO");
    } else {
      io.stop();
      Logger.getInstance().recordOutput("Elevator Mode", "OFF");
    }

    if (io.getLimitSwitch()) {
      zeroMotor();
    }

    updateMech2d();

    Logger.getInstance().recordOutput("Elevator Mech 2D", mech2d);
    Logger.getInstance()
        .recordOutput(
            "Elevator Goal", Constants.ElevatorConstants.PIDController.getGoal().position);
    Logger.getInstance()
        .recordOutput(
            "Elevator Setpoint", Constants.ElevatorConstants.PIDController.getSetpoint().position);
    Logger.getInstance().recordOutput("Elevator Enabled", enabled);
    Logger.getInstance().recordOutput("Is Zeroing", isZeroing);

    // We might have accidentaly tuned elevator pid with this call on, which modifies the state of
    // the pid controller
    // Basically, dont remove this line it's load bearing
    SmartDashboard.putNumber(
        "elevator pid output",
        Constants.ElevatorConstants.PIDController.calculate(getExtensionInches()));
  }
}
