// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int driverControllerPort = 0;
  }

  public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d());

  public static final class Swerve {
    public static final int pigeonID = 1;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot
    public static final double wheelBase = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /* Swerve Kinematics 
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
     public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values 
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 0.1;//4.5; //TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity = 1.0; //10.0; //TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = 3;
        public static final int angleMotorID = 4;
        public static final int canCoderID = 2;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(29.2);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = 1;
        public static final int angleMotorID = 2;
        public static final int canCoderID = 1;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(257.8);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
    
    /* Back Left Module - Module 2 */
    public static final class Mod2 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = 7;
        public static final int angleMotorID = 8;
        public static final int canCoderID = 4;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(24.6);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = 5;
        public static final int angleMotorID = 6;
        public static final int canCoderID = 3;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(186.5);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
    public static final double maxSpeedMetersPerSecond = 3;
    public static final double maxAccelerationMetersPerSecondSquared = 3;
    public static final double maxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double maxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final PathConstraints autoConstraints = new PathConstraints(maxSpeedMetersPerSecond, maxAccelerationMetersPerSecondSquared);

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints thetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            maxAngularSpeedRadiansPerSecond, maxAngularSpeedRadiansPerSecondSquared);
  }
  
  public static final class ElevatorConstants {
    public static final int elevatorMotorID = 0;
    public static final double elevatorGearRatio = 0.0;
    public static final ElevatorFeedforward feedforward = new ElevatorFeedforward(0.0, 0.0, 0.0);
    public static final TrapezoidProfile.Constraints elevatorConstraints = new TrapezoidProfile.Constraints(0.0,0.0);
    public static final ProfiledPIDController PIDController = new ProfiledPIDController(0.0, 0.0, 0.0, elevatorConstraints);
  }

  public static final class RotatingArmConstants {
    public static final int rotatingArmMotorID = 0;
    public static final double rotatingArmGearRatio = 0.0;
    public static final ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.0, 0.0);
    public static final TrapezoidProfile.Constraints rotatingArmConstraints = new TrapezoidProfile.Constraints(0.0,0.0);
    public static final ProfiledPIDController PIDController = new ProfiledPIDController(0.0, 0.0, 0.0, rotatingArmConstraints);
  }
}
