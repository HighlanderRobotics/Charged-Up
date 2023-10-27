package frc.robot.subsystems.Swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.choreolib.ChoreoSwerveControllerCommand;
import frc.lib.choreolib.ChoreoTrajectory;
import frc.robot.Constants;
import frc.robot.Constants.Grids;
import frc.robot.Constants.ScoringPositions;
import frc.robot.Constants.SimMode;
import frc.robot.PathPointOpen;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Vision.VisionHelper;
import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.subsystems.Vision.VisionIO.VisionIOInputs;
import frc.robot.subsystems.Vision.VisionIOReal;
import frc.robot.subsystems.Vision.VisionIOSim;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

/** SDS Mk4i Drivetrain */
public class SwerveSubsystem extends SubsystemBase {
  // degrees in radians
  public PIDController xBallanceController = new PIDController(0.01, 0, 0.1);
  public PIDController yBallanceController = new PIDController(1.0, 0, 0.5);
  public SwerveDrivePoseEstimator poseEstimator;
  public SwerveDriveOdometry wheelOnlyOdo;
  public SwerveModuleIO[] swerveMods;
  public SwerveModuleIOInputsAutoLogged[] inputs;
  public GyroIO gyroIO;
  public GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  public double simHeading = 0.0;
  public VisionHelper loggedEstimator;

  public Field2d field = new Field2d();

  private VisionIO visionIO =
      Robot.isReal()
          ? new VisionIOReal()
          : (Constants.SIM_MODE.equals(Constants.SimMode.SIM)
              ? new VisionIOSim()
              : new VisionIO() {

                @Override
                public void updateInputs(VisionIOInputs inputs, Pose3d robotPose) {}
              });
  private VisionIOInputs visionIOInputs = new VisionIOInputs();
  private AprilTagFieldLayout tagFieldLayout;

  public boolean hasResetOdometry = false;

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

  public Pose2d pose = new Pose2d();
  public boolean nearestGoalIsCone = true;
  public double extensionInches = 0;
  private double lastRoll = 0;
  private double rollRate = 0;
  private LinearFilter rollFilter = LinearFilter.singlePoleIIR(0.5, 0.020);
  public ElevatorSubsystem.ScoringLevels extensionLevel = ElevatorSubsystem.ScoringLevels.L2;

  public boolean lockOutSwerve = false;

  private ArrayList<Pose2d> dashboardFieldVisionPoses = new ArrayList<>();
  private ArrayList<Pose2d> dashboardFieldTapePoses = new ArrayList<>();

  public ProfiledPIDController headingController =
      new ProfiledPIDController(1.2, 0, 0.1, new Constraints(Math.PI * 4, Math.PI * 6));

  public SwerveSubsystem(SwerveModuleIO[] swerveIO, GyroIO gyroIO) {
    this.gyroIO = gyroIO;

    headingController.enableContinuousInput(0, Math.PI * 2);
    headingController.setTolerance(0.2);
    swerveMods = swerveIO;

    inputs =
        new SwerveModuleIOInputsAutoLogged[] {
          new SwerveModuleIOInputsAutoLogged(),
          new SwerveModuleIOInputsAutoLogged(),
          new SwerveModuleIOInputsAutoLogged(),
          new SwerveModuleIOInputsAutoLogged()
        };

    inputs[1].moduleNumber = 1;
    inputs[2].moduleNumber = 2;
    inputs[3].moduleNumber = 3;
    /*
     * By pausing init for a second before setting module offsets, we avoid a bug
     * with inverting motors.
     * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
     */
    Timer.delay(1.0);
    resetModulesToAbsolute();

    Vector<N3> odoStdDevs = VecBuilder.fill(0.3, 0.3, 0.01);
    Vector<N3> visStdDevs = VecBuilder.fill(1.3, 1.3, 3.3);

    poseEstimator =
        new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getYaw(),
            getModulePositions(),
            new Pose2d(),
            odoStdDevs,
            visStdDevs);

    wheelOnlyOdo =
        new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());

    List<Pose2d> tapePoses = new ArrayList<>();

    for (var goal : Grids.midTranslations) {
      System.out.println(goal.toString());
      tapePoses.add(new Pose2d(goal, new Rotation2d()));
    }
    for (var goal : Grids.highTranslations) {
      System.out.println(goal.toString());
      tapePoses.add(new Pose2d(goal, new Rotation2d()));
    }

    field.getObject("tape targets").setPoses(tapePoses);

    PPSwerveControllerCommand.setLoggingCallbacks(
        (traj) -> Logger.getInstance().recordOutput("Active Trajectory", traj),
        (target) -> Logger.getInstance().recordOutput("Active Trajectory Target", target),
        null,
        null);

    try {
      tagFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  /**
   * Set the modules to the correct state based on a desired translation and rotation, either field
   * or robot relative and either open or closed loop
   */
  public void drive(
      Translation2d translation,
      double rotation,
      boolean fieldRelative,
      boolean isOpenLoop,
      boolean useAlliance) {
    Pose2d velPose = new Pose2d(translation.times(0.02), new Rotation2d(rotation * 0.02));
    Twist2d velTwist = new Pose2d().log(velPose);
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    velTwist.dx / 0.02,
                    velTwist.dy / 0.02,
                    velTwist.dtheta / 0.02,
                    useAlliance && DriverStation.getAlliance() == DriverStation.Alliance.Red
                        ? getYaw().rotateBy(Rotation2d.fromRadians(Math.PI))
                        : getYaw())
                : new ChassisSpeeds(
                    velTwist.dx / 0.02, velTwist.dy / 0.02, velTwist.dtheta / 0.02));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    double[] logModuleStates = new double[8];
    for (int i = 0; i < 4; i++) {
      SwerveModuleIO mod = swerveMods[i];
      mod.setDesiredState(swerveModuleStates[(int) inputs[i].moduleNumber], isOpenLoop);
      logModuleStates[2 * i] = swerveModuleStates[i].angle.getRadians();
      logModuleStates[(2 * i) + 1] = swerveModuleStates[i].speedMetersPerSecond;
    }
    Logger.getInstance().recordOutput("Swerve Desired States", logModuleStates);
    chassisSpeeds = new ChassisSpeeds(velTwist.dx, velTwist.dy, velTwist.dtheta);
  }

  /** Generates a Command that consumes an X, Y, and Theta input supplier to drive the robot */
  public CommandBase driveCommand(
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier omega,
      boolean fieldRelative,
      boolean isOpenLoop,
      boolean useAlliance) {
    return new RunCommand(
        () ->
            drive(
                new Translation2d(x.getAsDouble(), y.getAsDouble())
                    .times(Constants.Swerve.maxSpeed),
                omega.getAsDouble() * Constants.Swerve.maxAngularVelocity,
                fieldRelative,
                isOpenLoop,
                useAlliance),
        this);
  }

  public Command headingLockDriveCommand(
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier theta,
      boolean fieldRelative,
      boolean isOpenLoop) {
    return driveCommand(
        x,
        y,
        () -> headingController.calculate(getYaw().getRadians(), theta.getAsDouble()),
        fieldRelative,
        isOpenLoop,
        false);
  }

  public Command poseLockDriveCommand(
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier theta,
      boolean fieldRelative,
      boolean isOpenLoop) {
    return new InstantCommand(
            () -> {
              Constants.AutoConstants.xController.reset(getPose().getX());
              Constants.AutoConstants.yController.reset(getPose().getY());
              headingController.reset(getYaw().getRadians() % (Math.PI * 2));
              headingController.setGoal(theta.getAsDouble());
            })
        .andThen(
            driveCommand(
                    () ->
                        deadband(
                            Constants.AutoConstants.xController.calculate(
                                pose.getX(), x.getAsDouble()),
                            0.05),
                    () ->
                        deadband(
                            Constants.AutoConstants.yController.calculate(
                                pose.getY(), y.getAsDouble()),
                            0.05),
                    () ->
                        deadband(
                            headingController.calculate(
                                pose.getRotation().getRadians() % (2 * Math.PI)),
                            0.05),
                    fieldRelative,
                    isOpenLoop,
                    false)
                .alongWith(
                    new PrintCommand(pose.getX() + " x"),
                    new PrintCommand(pose.getY() + " y"),
                    new PrintCommand(headingController.getPositionError() + " heading error")));
  }

  /** Generates a Command that consumes a PathPlanner path and follows it */
  public Command followPathCommand(PathPlannerTrajectory path) {
    return new SwerveControllerCommand(
        path,
        () -> getPose(),
        Constants.Swerve.swerveKinematics,
        new HolonomicDriveController(
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController,
                0,
                0,
                Constants.AutoConstants.thetaControllerConstraints)),
        (states) -> setModuleStates(states),
        this);
  }

  public PathPlannerTrajectory getPathBetweenTwoPoints(PathPoint start, PathPoint end) {
    return getPathBetweenTwoPoints(
        new PathConstraints(
            Constants.AutoConstants.maxSpeedMetersPerSecond,
            Constants.AutoConstants.maxAccelerationMetersPerSecondSquared),
        start,
        end);
  }

  public PathPlannerTrajectory getPathBetweenTwoPoints(
      PathConstraints constraints, PathPoint start, PathPoint end) {
    var path = PathPlanner.generatePath(constraints, start, end);
    return path;
  }

  public PathPlannerTrajectory getPathToPoint(PathPoint end) {
    field.getObject("starting pose").setPose(getPose());
    return getPathBetweenTwoPoints(
        PathPoint.fromCurrentHolonomicState(getPose(), chassisSpeeds), end);
  }

  public PathPointOpen getNearestGoal() {
    return getNearestGoal(getPose(), DriverStation.getAlliance());
  }

  public PathPointOpen getNearestGoal(Pose2d pose, Alliance alliance) {
    PathPointOpen output = null;
    Color8Bit color = null;
    double distance = Double.MAX_VALUE;
    if (alliance == Alliance.Blue) {
      for (PathPointOpen point : ScoringPositions.bluePositionsList) {
        double currentDistance =
            Math.sqrt(
                Math.pow(pose.getY() - point.getTranslation2d().getY(), 2)
                    + Math.pow(pose.getX() - point.getTranslation2d().getX(), 2));
        if (currentDistance < distance) {
          distance = currentDistance;
          output = point;
          color = Constants.lights.get(output);
        }
      }
    }
    if (alliance == Alliance.Red) {
      for (PathPointOpen point : ScoringPositions.redPositionsList) {
        double currentDistance =
            Math.sqrt(
                Math.pow(pose.getY() - point.getTranslation2d().getY(), 2)
                    + Math.pow(pose.getX() - point.getTranslation2d().getX(), 2));
        if (currentDistance < distance) {
          distance = currentDistance;
          output = point;
          color = Constants.lights.get(output);
        }
      }
    }
    field.getObject("goal").setPose(new Pose2d(output.getTranslation2d(), output.getRotation2d()));
    return output;
  }

  public double getNearestGoalDistance() {
    return pose.getTranslation().getDistance(getNearestGoal().getTranslation2d());
  }

  public boolean checkIfConeGoal(
      PathPointOpen goal) { // this doesn't apply for level 1 scoring positions
    if ((Constants.ScoringPositions.bluePositionsList.indexOf(goal) % 3) == 1
        || (Constants.ScoringPositions.redPositionsList.indexOf(goal) % 3)
            == 1) { // then its a cube goal
      return false;
    } else { // then its a cone goal
      return true;
    }
  }

  public double getExtension(ElevatorSubsystem.ScoringLevels level, boolean isCone) {
    if (isCone) {
      return level.getConeInches();
    } else {
      return level.getCubeInches();
    }
  }

  public SwerveAutoBuilder autoBuilder(Map<String, Command> eventMap) {
    return new SwerveAutoBuilder(
        () -> getPose(), // Pose2d supplier
        (Pose2d pose) ->
            resetOdometry(pose), // Pose2d consumer, used to reset odometry at the beginning of auto
        new PIDConstants(
            Constants.AutoConstants.kPXController,
            0.0,
            0.0), // PID constants to correct for translation error (used to create the X and Y
        // PID
        // controllers)
        new PIDConstants(
            Constants.AutoConstants.kPThetaController,
            0.0,
            0.0), // PID constants to correct for rotation error (used to create the rotation
        // controller)
        (ChassisSpeeds speeds) ->
            this.drive(
                new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                speeds.omegaRadiansPerSecond,
                false,
                false,
                false), // Module states consumer used to output to the drive subsystem
        eventMap,
        false, // Should the path be automatically mirrored depending on alliance color.
        // Optional,
        // defaults to true
        this // The drive subsystem. Used to properly set the requirements of path following
        // commands
        );
  }

  public Command choreoTrajFollow(ChoreoTrajectory traj, boolean shouldStop) {
    return new InstantCommand(
            () ->
                resetOdometry(
                    traj.sample(0, DriverStation.getAlliance() == Alliance.Red).getPose()))
        .andThen(
            new PrintCommand("! traj start"),
            new ChoreoSwerveControllerCommand(
                traj,
                this::getPose,
                new PIDController(
                    Constants.AutoConstants.kPXController,
                    0.0,
                    0.0), // PID constants to correct for translation error (used to create the X
                // and Y
                // PID
                // controllers)
                new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0),
                new PIDController(
                    Constants.AutoConstants.kPThetaController,
                    0.0,
                    0.0), // PID constants to correct for rotation error (used to create the
                // rotation
                // controller)
                (ChassisSpeeds speeds) ->
                    this.drive(
                        new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                        speeds.omegaRadiansPerSecond,
                        false,
                        false,
                        false),
                true,
                this),
            driveCommand(() -> 0, () -> 0, () -> 0, false, false, false)
                .until(() -> true)
                .unless(() -> !shouldStop));
  }

  public Command choreoTrajFollow(ChoreoTrajectory traj) {
    return choreoTrajFollow(traj, true);
  }

  public CommandBase autoBalanceVelocity() {
    return autoBalance()
        .until(
            () -> {
              return Math.abs(rollRate) > 2.5;
            })
        .andThen(
            new WaitCommand(0.25)
                .raceWith(driveCommand(() -> 0, () -> 0, () -> 0, false, false, false)))
        .repeatedly();
  }

  public CommandBase autoBalance() {
    return driveCommand(
        () -> {
          if (gyroInputs.rollDegrees > 11.0) {
            lockOutSwerve = false;
            return -0.1;
          } else if (gyroInputs.rollDegrees < -11.0) {
            lockOutSwerve = false;
            return 0.1;
          } else {
            lockOutSwerve = true;
            return 0.0;
          }
        },
        () -> 0,
        () -> 0,
        false,
        false,
        false);
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (int i = 0; i < 4; i++) {
      SwerveModuleIO mod = swerveMods[i];
      mod.setDesiredState(desiredStates[(int) inputs[i].moduleNumber], false);
    }
  }

  /** Return the pose of the drivebase, as estimated by the pose estimator. */
  public Pose2d getPose() {
    return wheelOnlyOdo.getPoseMeters();
    // Use this if we want to use vision
    // return poseEstimator.getEstimatedPosition();
  }

  /** Resets the pose estimator to the given pose */
  public void resetOdometry(Pose2d pose) {
    hasResetOdometry = true;
    zeroGyro(pose.getRotation().getDegrees());
    poseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
    wheelOnlyOdo.resetPosition(pose.getRotation(), getModulePositions(), pose);
    System.out.println("odometry reset " + pose.toString());
  }

  /**
   * @return the current state of each of the swerve modules, including current speed
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModuleIOInputsAutoLogged mod : inputs) {
      states[(int) mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  /**
   * @return the state of each of the swerve modules, including total distance
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModuleIOInputsAutoLogged mod : inputs) {
      positions[(int) mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  /** Resets the gyro to a heading of 0 */
  public void zeroGyro() {
    zeroGyro(0);
  }

  /** Resets the gyro to a given heading */
  public void zeroGyro(double angle) {
    gyroIO.resetHeading(angle);
    simHeading = angle;
  }

  /**
   * @return the yaw of the drive base, based on the gyro's rotation
   */
  public Rotation2d getYaw() {
    if (Robot.isReal() || Constants.SIM_MODE == SimMode.REPLAY) {
      return (Constants.Swerve.invertGyro)
          ? Rotation2d.fromDegrees(360 - gyroInputs.headingDegrees)
          : Rotation2d.fromDegrees(gyroInputs.headingDegrees);
    }
    return Rotation2d.fromDegrees(simHeading);
  }

  /** Resets the encoders on all swerve modules to the cancoder values */
  public void resetModulesToAbsolute() {
    for (SwerveModuleIO module : swerveMods) {
      module.resetToAbsolute();
    }
  }

  public double deadband(double value, double minumum) {
    if (Math.abs(value) < minumum) {
      return 0;
    }

    return value;
  }

  public void setLevel(ElevatorSubsystem.ScoringLevels level, boolean isCone) {
    extensionLevel = level;
    extensionInches = getExtension(level, isCone);
  }

  public ElevatorSubsystem.ScoringLevels getLevel() {
    return extensionLevel;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < swerveMods.length; i++) {
      inputs[i] = swerveMods[i].updateInputs();
      Logger.getInstance().processInputs("Swerve Module " + i, inputs[i]);
    }

    gyroInputs = gyroIO.updateInputs();
    Logger.getInstance().processInputs("Gyro", gyroInputs);
    simHeading += Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond);

    Logger.getInstance()
        .recordOutput(
            "Swerve States",
            new double[] {
              inputs[0].getState().angle.getRadians(),
              inputs[0].getState().speedMetersPerSecond,
              inputs[1].getState().angle.getRadians(),
              inputs[1].getState().speedMetersPerSecond,
              inputs[2].getState().angle.getRadians(),
              inputs[2].getState().speedMetersPerSecond,
              inputs[3].getState().angle.getRadians(),
              inputs[3].getState().speedMetersPerSecond
            });
    Logger.getInstance()
        .recordOutput(
            "Swerve Pose",
            new double[] {
              getPose().getX(), getPose().getY(), getPose().getRotation().getRadians()
            });
    Logger.getInstance().recordOutput("Swerve Sim Heading", simHeading);
    Logger.getInstance().recordOutput("Get Yaw", getYaw().getRadians());
    Logger.getInstance()
        .recordOutput(
            "Swerve Desired Speeds",
            new double[] {
              chassisSpeeds.vxMetersPerSecond,
              chassisSpeeds.vyMetersPerSecond,
              chassisSpeeds.omegaRadiansPerSecond
            });

    if (Robot.isReal() || Constants.SIM_MODE == SimMode.REPLAY) {
      pose = poseEstimator.update(getYaw(), getModulePositions());
      wheelOnlyOdo.update(getYaw(), getModulePositions());
    } else {
      pose = poseEstimator.update(Rotation2d.fromDegrees(simHeading), getModulePositions());
      wheelOnlyOdo.update(Rotation2d.fromDegrees(simHeading), getModulePositions());
    }

    visionIO.updateInputs(visionIOInputs, new Pose3d(pose));
    Logger.getInstance().processInputs("Vision", visionIOInputs);
    PhotonPipelineResult result =
        new PhotonPipelineResult(visionIOInputs.timeSinceLastTimestamp, visionIOInputs.targets);
    result.setTimestampSeconds(visionIOInputs.timestamp);
    
    try {
      var visionMeasurement =
          VisionHelper.update(
                  result, tagFieldLayout, PoseStrategy.MULTI_TAG_PNP, PoseStrategy.LOWEST_AMBIGUITY)
              .get()
              .estimatedPose;
      Logger.getInstance().recordOutput("Vision Pose", visionMeasurement);
      poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), visionIOInputs.timestamp);
    } catch (NoSuchElementException e) {}

    double[] apriltagX = new double[visionIOInputs.targets.size() * 4];
    double[] apriltagY = new double[visionIOInputs.targets.size() * 4];
    for (int i = 0; i < visionIOInputs.targets.size(); i++) {
      var target = visionIOInputs.targets.get(i);
      for (int j = 0; j < 4; j++) {
        apriltagX[(4 * i) + j] = target.getDetectedCorners().get(j).x;
        apriltagY[(4 * i) + j] = target.getDetectedCorners().get(j).y;
      }
    }

    Logger.getInstance().recordOutput("Apriltag X Dash", apriltagX);
    Logger.getInstance().recordOutput("Apriltag Y Dash", apriltagY);

    Logger.getInstance()
        .recordOutput(
            "Kalman Pose",
            new Pose2d(
                poseEstimator.getEstimatedPosition().getTranslation(),
                Rotation2d.fromDegrees(
                    poseEstimator.getEstimatedPosition().getRotation().getDegrees())));

    field.setRobotPose(getPose());
    field.getObject("odo only pose").setPose(wheelOnlyOdo.getPoseMeters());
    field.getObject("fused pose").setPose(poseEstimator.getEstimatedPosition());
    field.getObject("latest tag vision pose").setPoses(dashboardFieldVisionPoses);
    dashboardFieldVisionPoses.clear();
    dashboardFieldTapePoses.clear();
    SmartDashboard.putData(field);
    SmartDashboard.putBoolean("Has reset", hasResetOdometry);

    if (DriverStation.isDisabled()) {
      hasResetOdometry = false;
    }
    pose = getPose();
    nearestGoalIsCone = checkIfConeGoal(getNearestGoal());
    double filteredRoll = rollFilter.calculate(gyroInputs.rollDegrees);
    rollRate = (filteredRoll - lastRoll) / 0.020;
    lastRoll = filteredRoll;
  }
}
