// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

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
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.lib.components.HighlanderFalcon;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import static frc.robot.subsystems.ElevatorSubsystem.Level;

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

  public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(
    new Translation3d(0, Units.inchesToMeters(13.5), 0),
    new Rotation3d(0, 0, -Math.PI / 2));

  public static final class Swerve {
    public static final int pigeonID = 1;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants chosenModule =
        COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(22.7);
    public static final double wheelBase = Units.inchesToMeters(24.5);
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
    public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

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
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(76.3 + 180);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
    
    /* Back Left Module - Module 2 */
    public static final class Mod2 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = 7;
        public static final int angleMotorID = 8;
        public static final int canCoderID = 4;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(24.6 - 90);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = 5;
        public static final int angleMotorID = 6;
        public static final int canCoderID = 3;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(188.3);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
    public static final double maxSpeedMetersPerSecond = 3;
    public static final double maxAccelerationMetersPerSecondSquared = 3;
    public static final double maxAngularSpeedRadiansPerSecond = Math.PI * 2;
    public static final double maxAngularSpeedRadiansPerSecondSquared = Math.PI * 2;

    public static final PathConstraints autoConstraints = new PathConstraints(maxSpeedMetersPerSecond, maxAccelerationMetersPerSecondSquared);

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 0.5;
    public static final double kDThetaController = 0.1;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints thetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            maxAngularSpeedRadiansPerSecond, maxAngularSpeedRadiansPerSecondSquared);
  }

  public static final PathPointOpen blue0 = new PathPointOpen (new Translation2d(1.74, 0.43), Rotation2d.fromDegrees(0));
  public static final PathPointOpen blue1 = new PathPointOpen (new Translation2d(1.74, 1.03), Rotation2d.fromDegrees(0));
  public static final PathPointOpen blue2 = new PathPointOpen (new Translation2d(1.74, 1.63), Rotation2d.fromDegrees(0));
  public static final PathPointOpen blue3 = new PathPointOpen (new Translation2d(1.74, 2.23), Rotation2d.fromDegrees(0));
  public static final PathPointOpen blue4 = new PathPointOpen (new Translation2d(1.74, 2.83), Rotation2d.fromDegrees(0));
  public static final PathPointOpen blue5 = new PathPointOpen (new Translation2d(1.74, 3.43), Rotation2d.fromDegrees(0));
  public static final PathPointOpen blue6 = new PathPointOpen (new Translation2d(1.74, 4.03), Rotation2d.fromDegrees(0));
  public static final PathPointOpen blue7 = new PathPointOpen (new Translation2d(1.74, 4.63), Rotation2d.fromDegrees(0));
  public static final PathPointOpen blue8 = new PathPointOpen (new Translation2d(1.74, 5.23), Rotation2d.fromDegrees(0));  
  
  public static final PathPointOpen red0 = new PathPointOpen (new Translation2d(14.6, 0.43), Rotation2d.fromDegrees(0));
  public static final PathPointOpen red1 = new PathPointOpen (new Translation2d(14.6, 1.03), Rotation2d.fromDegrees(0));
  public static final PathPointOpen red2 = new PathPointOpen (new Translation2d(14.6, 1.63), Rotation2d.fromDegrees(0));
  public static final PathPointOpen red3 = new PathPointOpen (new Translation2d(14.6, 2.23), Rotation2d.fromDegrees(0));
  public static final PathPointOpen red4 = new PathPointOpen (new Translation2d(14.6, 2.83), Rotation2d.fromDegrees(0));
  public static final PathPointOpen red5 = new PathPointOpen (new Translation2d(14.6, 3.43), Rotation2d.fromDegrees(0));
  public static final PathPointOpen red6 = new PathPointOpen (new Translation2d(14.6, 4.03), Rotation2d.fromDegrees(0));
  public static final PathPointOpen red7 = new PathPointOpen (new Translation2d(14.6, 4.63), Rotation2d.fromDegrees(0));
  public static final PathPointOpen red8 = new PathPointOpen (new Translation2d(14.6, 5.23), Rotation2d.fromDegrees(0));  

  public static final class ScoringPositions {
    public static final HashMap<String, PathPointOpen> positions = new HashMap<>();
    static {
      positions.put("blue0", blue0);
      positions.put("blue1", blue1);
      positions.put("blue2", blue2);
      positions.put("blue3", blue3);
      positions.put("blue4", blue4);
      positions.put("blue5", blue5);
      positions.put("blue6", blue6);
      positions.put("blue7", blue7);
      positions.put("blue8", blue8);

      positions.put("red0", red0);
      positions.put("red1", red1);
      positions.put("red2", red2);
      positions.put("red3", red3);
      positions.put("red4", red4);
      positions.put("red5", red5);
      positions.put("red6", red6);
      positions.put("red7", red7);
      positions.put("red8", red8);
    }
    public static final List<PathPointOpen > bluePositionsList = new ArrayList<>();
    static {
      bluePositionsList.add(blue0);
      bluePositionsList.add(blue1);
      bluePositionsList.add(blue2);
      bluePositionsList.add(blue3);
      bluePositionsList.add(blue4);
      bluePositionsList.add(blue5);
      bluePositionsList.add(blue6);
      bluePositionsList.add(blue7);
      bluePositionsList.add(blue8);
    }
    public static final List<PathPointOpen > redPositionsList = new ArrayList<>();
    static {
      redPositionsList.add(red0);
      redPositionsList.add(red1);
      redPositionsList.add(red2);
      redPositionsList.add(red3);
      redPositionsList.add(red4);
      redPositionsList.add(red5);
      redPositionsList.add(red6);
      redPositionsList.add(red7);
      redPositionsList.add(red8);
    }
  }
  public static final PathPointOpen blueSubstation = new PathPointOpen(new Translation2d(0.85, 7.45), Rotation2d.fromDegrees(0));
  public static final PathPointOpen redSubstation = new PathPointOpen(new Translation2d(15.7, 7.45), Rotation2d.fromDegrees(0));
 
  public static final float elevatorMargin = 0; //TODO: i have no idea what the actual number is
  public static final float topCubeGoal = 90;
  public static final float midCubeGoal = 60;
  public static final float bottomGoal = 0;
  
  public static final class ElevatorConstants {
    public static final int elevatorMotorID = 25;
    public static final int elevatorFollowerID = 26;
    // TODO: check this
    public static final double elevatorGearRatio = 5.45;
    public static final ElevatorFeedforward feedforward = new ElevatorFeedforward(0.4887, 0.33984, 0.0053894, 0.007808);
    public static final TrapezoidProfile.Constraints elevatorConstraints = new TrapezoidProfile.Constraints(5.0,10.0);
    public static final ProfiledPIDController PIDController = new ProfiledPIDController(0.032409, 0.0, 0.032608, elevatorConstraints);

    static {
      PIDController.setTolerance(
        HighlanderFalcon.radToNative(0.1), //TODO: is this good?
        HighlanderFalcon.rpmToNative(1.0));
    }

    public static final double elevatorAngleRad = Math.toRadians(41);
    public static final double maxExtensionInches = 72;
    public static final Translation2d elevatorOffset = new Translation2d(-5.1, 13.6); // TODO: find actual numbers for this
    // Positions that the end effector needs to be in to score
    // TODO: tune
    public static final Translation2d l1Translation = new Translation2d(20.0, 12.0); 
    //x is distance forward from robot center, y is distance up from floor

    public static final Translation2d l2TranslationCones = new Translation2d(34.0, 34.0);
    public static final Translation2d l3TranslationCones = new Translation2d(50.0, 46.0);
    
    public static final Translation2d l2TranslationCubes = new Translation2d(34.0, 34.0);
    public static final Translation2d l3TranslationCubes = new Translation2d(50.0, 44.0);

    public static final Translation2d humanPlayerTranslation = new Translation2d(30, 37.325); // TODO: Find

    public static final Translation2d defaultPosition = new Translation2d(5, -5); //TODO: Find

    public static Translation2d getGoalTranslationCones(Level level) {
      switch (level) {
        case L2:
          return l2TranslationCones;
        case L3:
          return l3TranslationCones;
        case L1:
          return l1Translation;
        case HUMAN_PLAYER:
          return humanPlayerTranslation;
        default:
          return defaultPosition;
      }
    }
    
    public static Translation2d getGoalTranslationCubes(Level level) {
      switch (level) {
        case L2:
          return l2TranslationCubes;
        case L3:
          return l3TranslationCubes;
        case L1:
          return l1Translation;
        case HUMAN_PLAYER:
          return humanPlayerTranslation;
        default:
          return defaultPosition;
      }
    }

    public static final Constraints elevatorArmSystemConstraints = new Constraints(10.0, 10.0);
    public static final double elevatorSpoolCircumference = 1.751 * Math.PI;
  }

  public static final class ArmConstants {
    public static final int rotatingArmMotorID = 27;
    // TODO: Check with actual robot
    public static final double rotatingArmGearRatio = (12.0 / 18.0) * (1.0 / 45.0) * (1.0 / 1.0);
    public static final ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.0, 0.0);
    public static final TrapezoidProfile.Constraints rotatingArmConstraints = new TrapezoidProfile.Constraints(0.0,0.0);
    public static final ProfiledPIDController PIDController = new ProfiledPIDController(0.0, 0.0, 0.0, rotatingArmConstraints);
    static {
      PIDController.setTolerance(
        HighlanderFalcon.radToNative(0.1), //TODO: is this good?
        HighlanderFalcon.rpmToNative(1.0));
    }
    public static final double rotatingArmLengthInches = 12.5;
    public static final double armOffset = -ElevatorConstants.elevatorAngleRad;

    public static final double armMinimumAngle = Math.toRadians(-20);
    public static final double armMaximumAngle = Math.toRadians(50);
  }
  
  /** Constants for simple mechanisms like intake, routing, grabber */
  public static final class MechanismConstants {
    public static final int intakeID = 20;
    public static final int intakeSolenoidForwardID = 2;
    public static final int intakeSolenoidBackwardID = 0;
    public static final double intakeTimeToExtend = 0.3; // TODO: find

    public static final int routingLeftID = 21;
    public static final int routingRightID = 22;
    public static final int routingConveyerID = 23;
    public static final double routingKP = 1; // TODO: tune
    public static final double conveyerKP = 1; // TODO: tune

    public static final int routingLimitSwitch = 2;
    public static final boolean isRoutingSwitchReversed = false; 

    public static final int grabberID = 24;
    public static final int grabberSolenoidFrontID = 1;
    public static final int grabberSolenoidBackID = 3;

    public static final int grabberLimitSwitch = 1;
    public static final boolean isGrabberSwitchReversed = true; 
  }
}
