package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/** SDS Mk4i Drivetrain */
public class SwerveSubsystem extends SubsystemBase {
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public Field2d field = new Field2d();

    private PhotonCamera camera;
    private PhotonPipelineResult result;
    private AprilTagFieldLayout fieldLayout;

    public SwerveSubsystem() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

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

        // TODO: Add stddev matrices
        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions(), new Pose2d());

        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (Exception e) {
            System.out.print("Failed to initialize apriltag layout");
        }
    }

    /** Set the modules to the correct state based on a desired translation and rotation, either field or robot relative and either open or closed loop */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /** Generates a Command that consumes an X, Y, and Theta input supplier to drive the robot */
    public Command driveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta, boolean fieldRelative, boolean isOpenLoop) {
        return new RunCommand(
            () -> drive(
                new Translation2d(x.getAsDouble() * x.getAsDouble() * Math.signum(x.getAsDouble()), y.getAsDouble() * y.getAsDouble() * Math.signum(y.getAsDouble())).times(Constants.Swerve.maxSpeed), 
                theta.getAsDouble() * Constants.Swerve.maxAngularVelocity, 
                fieldRelative, 
                isOpenLoop), 
                this);
    }

    /** Generates a Command that consumes a PathPlanner path and follows it */
    public Command followPathCommand(PathPlannerTrajectory path) {
        return new SwerveControllerCommand(
            path,
            () -> getPose(), Constants.Swerve.swerveKinematics,
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

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    /** Return the pose of the drivebase, as estimated by the pose estimator. */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Resets the pose estimator to the given pose */
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    /** Updates the pose estimator from a (presumably vision) measurement
     * Input is in the form of a list of pose2ds and a latency measurement
     */
    public void updateOdometry(Pair<List<Pose2d>, Double> data){
        System.out.println(data.getFirst());

        if (data != null) {
        field.getObject("Latest Vision Pose").setPoses(data.getFirst());
        SmartDashboard.putNumber("Latency", data.getSecond());
        for (Pose2d pose : data.getFirst()){
            // Data is in milliseconds, need to convert to seconds
            poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp() - (data.getSecond() / 1000));
            zeroGyro(pose.getRotation().getDegrees());
        }
        }
    }

    
  /**Processes the vision result.
   * 
   * @return a pair of the list of poses from each target, and the latency of the result
   */
  public Pair<List<Pose2d>, Double> getEstimatedPose(){
    // Only do work if we actually have targets, if we don't return null
    if (result.hasTargets()){
      // List that we're going to return later
      List<Pose2d> poses = new ArrayList<Pose2d>();
      // Loop through all the targets
      for (PhotonTrackedTarget target : result.getTargets()){
        // Use a switch statement to lookup the pose of the marker
        // Later will switch this to use wpilibs json file to lookup pose of marker
        Pose3d targetPose3d = new Pose3d();
        if (fieldLayout != null && fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
            targetPose3d = fieldLayout.getTagPose(target.getFiducialId()).get();
        } else if (fieldLayout == null) {
            System.out.println("No field layout");
            continue;
        } else {
            System.out.println("No tag found with that ID");
            continue;
        }
        // Reject targets with a high ambiguity. Threshold should be tuned
        if (target.getPoseAmbiguity() < 0.1) {
          // Calculate and add the pose to our list of poses
          // May need to invert the camera to robot transform?
          poses.add(getFieldToRobot(targetPose3d, Constants.CAMERA_TO_ROBOT, target.getBestCameraToTarget()).toPose2d());
        }
        // Return the list of poses and the latency
        return new Pair<>(poses, result.getLatencyMillis());
      }
    }
    // Returns null if no targets are found
    return null;
  }

    /**
     * Estimates the pose of the robot in the field coordinate system, given the id of the fiducial, the robot relative to the
     * camera, and the target relative to the camera.
     * Stolen from mdurrani834
     * @param tagPose Pose3d the field relative pose of the target
     * @param robotToCamera Transform3d of the robot relative to the camera. Origin of the robot is defined as the center.
     * @param cameraToTarget Transform3d of the target relative to the camera, returned by PhotonVision
     * @return Pose Robot position relative to the field.
     */
    private Pose3d getFieldToRobot(Pose3d tagPose, Transform3d robotToCamera, Transform3d cameraToTarget) {
        return tagPose.plus(cameraToTarget.inverse()).plus(robotToCamera.inverse()); 
    }   

    public double getCameraResultLatency(){
        return result.getLatencyMillis();
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
    private void resetModulesToAbsolute() {
        for (SwerveModule module: mSwerveMods) {
            module.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        poseEstimator.update(getYaw(), getModulePositions());  
        
        result = camera.getLatestResult();

        if (DriverStation.isDisabled()){
            resetModulesToAbsolute();
        }

        // Log swerve module information
        // May want to disable to conserve bandwidth
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        SmartDashboard.putNumber("Heading", getYaw().getDegrees());
    }
}