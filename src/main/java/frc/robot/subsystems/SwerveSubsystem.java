package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants.Grids;
import frc.robot.Constants.ScoringPositions;
import frc.robot.subsystems.ElevatorSubsystem.ScoringLevels;
import frc.robot.subsystems.ApriltagVisionSubsystem.VisionMeasurement;
import frc.robot.Constants;
import frc.robot.PathPointOpen;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

/** SDS Mk4i Drivetrain */
public class SwerveSubsystem extends SubsystemBase {
    //degrees in radians 
    public PIDController xBallanceController = new PIDController(0.01, 0, 0.1);
    public PIDController yBallanceController = new PIDController(1.0, 0, 0.5);
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveDriveOdometry wheelOnlyOdo;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public Field2d field = new Field2d();

    private boolean isInTapeMode = true;
    private PIDController tapeDriveAssistController = new PIDController(-0.02, 0, 0);

    private ApriltagVisionSubsystem apriltagVisionSubsystem = new ApriltagVisionSubsystem();

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

    public ProfiledPIDController headingController = new ProfiledPIDController(
        1.2, 
        0, 
        0.0,
        new Constraints(Math.PI * 2, Math.PI * 2));

    public TapeVisionSubsystem tapeVisionSubsystem = new TapeVisionSubsystem("limelight-left", Constants.leftCameraToRobot);
    double lastApriltagTime = 0;

    public SwerveSubsystem() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        headingController.enableContinuousInput(0, Math.PI * 2);
        headingController.setTolerance(0.15);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        Vector<N3> odoStdDevs = VecBuilder.fill(0.3, 0.3, 0.3);
        Vector<N3> visStdDevs = VecBuilder.fill(0.6, 0.6, 0.3);

        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics, 
            getYaw(), 
            getModulePositions(), 
            new Pose2d(),
            odoStdDevs,
            visStdDevs);

        wheelOnlyOdo = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
        
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
    }

    /** Set the modules to the correct state based on a desired translation and rotation, either field or robot relative and either open or closed loop */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, boolean useAlliance) {
        Pose2d velPose = new Pose2d(translation.times(0.02), new Rotation2d(rotation * 0.02));
        Twist2d velTwist = new Pose2d().log(velPose);
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    velTwist.dx / 0.02, 
                                    velTwist.dy / 0.02, 
                                    velTwist.dtheta /0.02, 
                                    useAlliance && DriverStation.getAlliance() == DriverStation.Alliance.Red
                                        ? getYaw() : getYaw()
                                )
                                : new ChassisSpeeds(
                                    velTwist.dx / 0.02, 
                                    velTwist.dy / 0.02, 
                                    velTwist.dtheta /0.02)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }

        chassisSpeeds = new ChassisSpeeds(velTwist.dx, velTwist.dy, velTwist.dtheta);
    }

    /** Generates a Command that consumes an X, Y, and Theta input supplier to drive the robot */
    public CommandBase driveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega, boolean fieldRelative, boolean isOpenLoop, boolean useAlliance) {
        return new RunCommand(
            () -> drive(
                new Translation2d(x.getAsDouble(), y.getAsDouble()).times(Constants.Swerve.maxSpeed), 
                omega.getAsDouble() * Constants.Swerve.maxAngularVelocity, 
                fieldRelative, 
                isOpenLoop,
                useAlliance), 
                this);
    }

    public Command headingLockDriveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta, boolean fieldRelative, boolean isOpenLoop) {
        return driveCommand(
            x, 
            y, 
            () -> headingController.calculate(getYaw().getRadians(), theta.getAsDouble()), 
            fieldRelative, 
            isOpenLoop,
            true);
    }
    public Command poseLockDriveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta, boolean fieldRelative, boolean isOpenLoop) {
        return new InstantCommand(
            () -> {Constants.AutoConstants.xController.reset(getPose().getX()); 
                Constants.AutoConstants.yController.reset(getPose().getY());
                headingController.reset(getYaw().getRadians() % (Math.PI * 2));
                headingController.setGoal(theta.getAsDouble());}).andThen(
            driveCommand(
                () -> deadband(Constants.AutoConstants.xController.calculate(pose.getX(), x.getAsDouble()), 0.05), 
                () -> deadband(Constants.AutoConstants.yController.calculate(pose.getY(), y.getAsDouble()), 0.05),
                () -> deadband(headingController.calculate(pose.getRotation().getRadians() % (2 * Math.PI)), 0.05),
                fieldRelative, 
                isOpenLoop,
                false).alongWith(
                    new PrintCommand(pose.getX() + " x"),
                    new PrintCommand(pose.getY() + " y"),
                    new PrintCommand(headingController.getPositionError() + " heading error")
                ));
    }

    /** Generates a Command that consumes a PathPlanner path and follows it */
    public Command followPathCommand(PathPlannerTrajectory path) {
        //field.getObject("path goal").setPoses(path.getEndState().poseMeters);
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
    public PathPlannerTrajectory getPathBetweenTwoPoints (PathPoint start, PathPoint end) {
        return getPathBetweenTwoPoints(new PathConstraints(Constants.AutoConstants.maxSpeedMetersPerSecond, Constants.AutoConstants.maxAccelerationMetersPerSecondSquared), start, end);
        
    }
    public PathPlannerTrajectory getPathBetweenTwoPoints (PathConstraints constraints, PathPoint start, PathPoint end) {
        var path = PathPlanner.generatePath(constraints, start, end);
        List<Pose2d> poses = new ArrayList<>();
        // for (State state : path.getStates()) {
        //     poses.add(state.poseMeters);
        // }
        //field.getObject("path").setPoses(poses);
        return path;
        
    }
    public PathPlannerTrajectory getPathToPoint (PathPoint end) {
        field.getObject("starting pose").setPose(getPose());
        return getPathBetweenTwoPoints(PathPoint.fromCurrentHolonomicState(getPose(), chassisSpeeds), end);
    }
    public PathPointOpen getNearestGoal () {
        return getNearestGoal(getPose(), DriverStation.getAlliance());
    }
    public PathPointOpen getNearestGoal (Pose2d pose, Alliance alliance) {
        PathPointOpen output = null;
        Color8Bit color = null;
        double distance = Double.MAX_VALUE;
        if (alliance == Alliance.Blue) {
            for (PathPointOpen point : ScoringPositions.bluePositionsList) {
                double currentDistance = Math.sqrt(Math.pow(pose.getY() - point.getTranslation2d().getY(), 2) +
                    Math.pow(pose.getX() - point.getTranslation2d().getX(), 2));
                if (currentDistance < distance) {
                    distance = currentDistance;
                    output = point;
                    color = Constants.lights.get(output);
                }
            }
        }
        if (alliance == Alliance.Red) {
            for (PathPointOpen point : ScoringPositions.redPositionsList) {
                double currentDistance = Math.sqrt(Math.pow(pose.getY() - point.getTranslation2d().getY(), 2) +
                    Math.pow(pose.getX() - point.getTranslation2d().getX(), 2));
                if (currentDistance < distance) {
                    distance = currentDistance;
                    output = point;
                    color = Constants.lights.get(output);
                }
            }
        }
        field.getObject("goal").setPose(new Pose2d(output.getTranslation2d(), output.getRotation2d()));
       // ledSubsystem.setBlinking(color, 1);
        return output;
    }
    public double getNearestGoalDistance () {
        return pose.getTranslation().getDistance(getNearestGoal().getTranslation2d());
    }
    public boolean checkIfConeGoal(PathPointOpen goal){ //this doesn't apply for level 1 scoring positions
        // System.out.println("index " + (Constants.ScoringPositions.bluePositionsList.indexOf(goal) % 3));
        // System.out.println("index " + (Constants.ScoringPositions.redPositionsList.indexOf(goal) % 3));
        if ((Constants.ScoringPositions.bluePositionsList.indexOf(goal) % 3) == 1 ||
        (Constants.ScoringPositions.redPositionsList.indexOf(goal) % 3) == 1){ //then its a cube goal
            return false;
        }
        else { //then its a cone goal
            return true;
        }
    }

    public double getExtension(ElevatorSubsystem.ScoringLevels level, boolean isCone) {
        // System.out.println(nearestGoalIsCone);
        if (isCone) {
            return level.getConeInches();
        } else {
            return level.getCubeInches();
        }
    }

    public SwerveAutoBuilder autoBuilder(Map<String, Command> eventMap){
        return
            new SwerveAutoBuilder(
                () -> getPose(), // Pose2d supplier
                (Pose2d pose) -> resetOdometry(pose), // Pose2d consumer, used to reset odometry at the beginning of auto
                Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
                new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
                new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
                this::setModuleStates, // Module states consumer used to output to the drive subsystem
                eventMap,
                false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                this // The drive subsystem. Used to properly set the requirements of path following commands
                );
    }

    public CommandBase autoBalanceVelocity() {
        return autoBalance().until(() -> {return Math.abs(rollRate) > 2.0;})
            .andThen(new WaitCommand(0.5).raceWith(driveCommand(() -> 0, () -> 0, () -> 0, false, false, false))).repeatedly();
    }
    
    public CommandBase autoBalance(){
        return driveCommand(
                () -> {
                    if (gyro.getRoll() > 11.0) {
                        lockOutSwerve = false;
                        return -0.1;
                    } else if (gyro.getRoll() < -11.0) {
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

    // public CommandBase disableGamePieceOverride() {
    //     return new InstantCommand(() -> isConeOveride = Optional.empty());
    // }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    /** Return the pose of the drivebase, as estimated by the pose estimator. */
    public Pose2d getPose() {
        // return wheelOnlyOdo.getPoseMeters();
        return poseEstimator.getEstimatedPosition();
    }

    /** Resets the pose estimator to the given pose */
    public void resetOdometry(Pose2d pose) {
        hasResetOdometry = true;
        zeroGyro(pose.getRotation().getDegrees());
        poseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
        wheelOnlyOdo.resetPosition(pose.getRotation(), getModulePositions(), pose);
        System.out.println("odometry reset " + pose.toString());
    }

    /** @return the current state of each of the swerve modules, including current speed */
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /** @return the state of each of the swerve modules, including total distance */
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /** Resets the gyro to a heading of 0 */
    public void zeroGyro(){
        zeroGyro(0);
    }

    /** Resets the gyro to a given heading */
    public void zeroGyro(double angle){
        gyro.setYaw(angle);
    }

    /** @return the yaw of the drive base, based on the gyro's rotation */
    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    /** Resets the encoders on all swerve modules to the cancoder values */
    public void resetModulesToAbsolute() {
        for (SwerveModule module: mSwerveMods) {
            module.resetToAbsolute();
        }
    }

    public double deadband(double value, double minumum) {
        if (Math.abs(value) < minumum) {
            return 0;
        }

        return value;
    }
    public void setLevel(ElevatorSubsystem.ScoringLevels level, boolean isCone){
        extensionLevel = level;
        extensionInches = getExtension(level, isCone);
    }
    public ElevatorSubsystem.ScoringLevels getLevel(){
        return extensionLevel;
    }

    @Override
    public void periodic(){
        pose = poseEstimator.update(getYaw(), getModulePositions()); 
        wheelOnlyOdo.update(getYaw(), getModulePositions());
        
        List<VisionMeasurement> visionMeasurements = apriltagVisionSubsystem.getEstimatedGlobalPose(pose);

        for (VisionMeasurement measurement : visionMeasurements) {
            dashboardFieldVisionPoses.add(measurement.estimation.estimatedPose.toPose2d());
            poseEstimator.addVisionMeasurement(
                measurement.estimation.estimatedPose.toPose2d(),
                measurement.estimation.timestampSeconds,
                measurement.confidence.times(1.0));
            lastApriltagTime = measurement.estimation.timestampSeconds;
        }
        
        // var tapeVisionMeasurements = tapeVisionSubsystem.getEstimatedPoses(poseEstimator.getEstimatedPosition());
        
        // if (Timer.getFPGATimestamp() - lastApriltagTime < 1.0) {
        //     for (var measurement : tapeVisionMeasurements.getFirst()) {
        //         dashboardFieldTapePoses.add(measurement);
        //         poseEstimator.addVisionMeasurement(measurement, tapeVisionMeasurements.getSecond(), Constants.PoseEstimator.TAPE_VISION_MEASUREMENT_STANDARD_DEVIATIONS);
        //     }
        // }
        // Log swerve module information
        // May want to disable to conserve bandwidth
        // for(SwerveModule mod : mSwerveMods){
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        // }

        SmartDashboard.putNumber("Heading", getYaw().getDegrees());
        field.setRobotPose(getPose());
        field.getObject("odo only pose").setPose(wheelOnlyOdo.getPoseMeters());
        field.getObject("fused pose").setPose(poseEstimator.getEstimatedPosition());
        field.getObject("latest tag vision pose").setPoses(dashboardFieldVisionPoses);
        field.getObject("latest tape vision pose").setPoses(dashboardFieldTapePoses);
        dashboardFieldVisionPoses.clear();
        dashboardFieldTapePoses.clear();
        SmartDashboard.putData(field);
        SmartDashboard.putBoolean("Has reset", hasResetOdometry);

        if (DriverStation.isDisabled()) {
            hasResetOdometry = false;
        }
        // getNearestGoal();
        // getPathToPoint(getNearestGoal());

        SmartDashboard.putNumber("x error", Constants.AutoConstants.xController.getPositionError());
        SmartDashboard.putNumber("y error", Constants.AutoConstants.yController.getPositionError());
        SmartDashboard.putNumber("x goal", Constants.AutoConstants.xController.getGoal().position);
        SmartDashboard.putNumber("Y goal", Constants.AutoConstants.yController.getGoal().position);
        SmartDashboard.putNumber("Heading goal", headingController.getGoal().position);
        SmartDashboard.putNumber("Heading error", headingController.getPositionError());
        SmartDashboard.putNumber("total error", getNearestGoalDistance());
        SmartDashboard.putBoolean("is cone goal", nearestGoalIsCone);
        SmartDashboard.putNumber("extension requested", getExtension(ScoringLevels.L2, true));
        SmartDashboard.putString("alliance", DriverStation.getAlliance().toString());
        SmartDashboard.putNumber("gyro roll", gyro.getRoll());
        SmartDashboard.putNumber("gyro pitch", gyro.getPitch());
        SmartDashboard.putNumber("swerve chassis speeds", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("balance pid out", xBallanceController.calculate(deadband(gyro.getRoll(), 6.0)));
        SmartDashboard.putBoolean("is in tape mode", isInTapeMode);
        SmartDashboard.putBoolean("should lock out", lockOutSwerve);        
        pose = getPose();
        nearestGoalIsCone = checkIfConeGoal(getNearestGoal());
        double filteredRoll = rollFilter.calculate(gyro.getRoll());
        rollRate = (filteredRoll - lastRoll) / 0.020;
        lastRoll = filteredRoll;

    }

    @Override
    public void simulationPeriodic() {
        Pose2d visSimRobotPose = new Pose2d(Units.inchesToMeters((60 + (Timer.getFPGATimestamp() * 15)) % 200), 0.513, new Rotation2d(3.3));
        field.getObject("vis sim robot pose").setPose(visSimRobotPose);
        tapeVisionSubsystem.updateSimCamera(visSimRobotPose);
        try {
            field.getObject("vis sim est pose").setPoses(tapeVisionSubsystem.getEstimatedPoses(visSimRobotPose).getFirst());
        } catch (Exception exception) {
            field.getObject("vis sim est pose").setPoses();
        }
        System.out.print("");
    }
}