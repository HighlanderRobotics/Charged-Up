// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.components.HighlanderFalcon;
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
    public static final int operatorControllerPort = 1;
  }

  public static final Transform3d rightCameraToRobot = new Transform3d(
    new Translation3d(
      Units.inchesToMeters(-8),
      Units.inchesToMeters(-9.75), 
      Units.inchesToMeters(-22.75)),
    new Rotation3d(0, 0, Units.degreesToRadians(-5)));
  public static final Transform3d leftCameraToRobot = new Transform3d(
    new Translation3d(
      Units.inchesToMeters(-8),
      Units.inchesToMeters(9.75), 
      Units.inchesToMeters(-22.75)),
    new Rotation3d(0, 0, Units.degreesToRadians(5)));

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
    public static final double maxSpeed = 4; //TODO: This must be tuned to specific robot
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
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(29.2 - 90);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { //TODO: This must be tuned to specific robot
        public static final int driveMotorID = 1;
        public static final int angleMotorID = 2;
        public static final int canCoderID = 1;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(76.3 + 90);
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
    public static final double maxAccelerationMetersPerSecondSquared = 5;
    public static final double maxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double maxAngularSpeedRadiansPerSecondSquared = Math.PI / 2;

    public static final PathConstraints autoConstraints = new PathConstraints(maxSpeedMetersPerSecond, maxAccelerationMetersPerSecondSquared);
    public static final Constraints constraints = new Constraints(maxSpeedMetersPerSecond, maxAccelerationMetersPerSecondSquared);

    public static final double kPXController = 4.0;
    public static final double kPYController = 4.0;
    public static final double kPThetaController = 1.3;
    public static final double kDThetaController = 0.0;

    public static final ProfiledPIDController xController = new ProfiledPIDController(kPXController, 0, 0, constraints);
    public static final ProfiledPIDController yController = new ProfiledPIDController(kPYController, 0, 0, constraints);

    public static final SimpleMotorFeedforward thetaFeedForward = new SimpleMotorFeedforward(0.24, 1.0e+3);
    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints thetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            maxAngularSpeedRadiansPerSecond, maxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class ScoringPositions {
    public static final PathPointOpen blue0 = new PathPointOpen (new Translation2d(1.7, 0.43), Rotation2d.fromDegrees(180));
    public static final PathPointOpen blue1 = new PathPointOpen (new Translation2d(1.7, 1.03), Rotation2d.fromDegrees(180));
    public static final PathPointOpen blue2 = new PathPointOpen (new Translation2d(1.7, 1.63), Rotation2d.fromDegrees(180));
    public static final PathPointOpen blue3 = new PathPointOpen (new Translation2d(1.7, 2.23), Rotation2d.fromDegrees(180));
    public static final PathPointOpen blue4 = new PathPointOpen (new Translation2d(1.7, 2.83), Rotation2d.fromDegrees(180));
    public static final PathPointOpen blue5 = new PathPointOpen (new Translation2d(1.7, 3.43), Rotation2d.fromDegrees(180));
    public static final PathPointOpen blue6 = new PathPointOpen (new Translation2d(1.7, 3.8), Rotation2d.fromDegrees(180));
    public static final PathPointOpen blue7 = new PathPointOpen (new Translation2d(1.7, 4.63), Rotation2d.fromDegrees(180));
    public static final PathPointOpen blue8 = new PathPointOpen (new Translation2d(1.7, 5.23), Rotation2d.fromDegrees(180));  
    
    public static final PathPointOpen red0 = new PathPointOpen (new Translation2d(15.0, 0.52), Rotation2d.fromDegrees(0));
    public static final PathPointOpen red1 = new PathPointOpen (new Translation2d(15.0, 1.03), Rotation2d.fromDegrees(0));
    public static final PathPointOpen red2 = new PathPointOpen (new Translation2d(15.0, 1.63), Rotation2d.fromDegrees(0));
    public static final PathPointOpen red3 = new PathPointOpen (new Translation2d(15.0, 2.23), Rotation2d.fromDegrees(0));
    public static final PathPointOpen red4 = new PathPointOpen (new Translation2d(15.0, 2.83), Rotation2d.fromDegrees(0));
    public static final PathPointOpen red5 = new PathPointOpen (new Translation2d(15.0, 3.3), Rotation2d.fromDegrees(0));
    public static final PathPointOpen red6 = new PathPointOpen (new Translation2d(15.0, 4.03), Rotation2d.fromDegrees(0));
    public static final PathPointOpen red7 = new PathPointOpen (new Translation2d(15.0, 4.63), Rotation2d.fromDegrees(0));
    public static final PathPointOpen red8 = new PathPointOpen (new Translation2d(15.0, 5.23), Rotation2d.fromDegrees(0));  

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
    public static final List<PathPointOpen> bluePositionsList = new ArrayList<>();
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
  public static final class ScoringLEDs {
    public static final Color8Bit goal0 = new Color8Bit(255, 0, 0); //same for red and blue goals
    public static final Color8Bit goal1 = new Color8Bit(255, 125, 0);
    public static final Color8Bit goal2 = new Color8Bit(255, 255, 0);
    public static final Color8Bit goal3 = new Color8Bit(0, 255, 0);
    public static final Color8Bit goal4 = new Color8Bit(0, 255, 255);
    public static final Color8Bit goal5 = new Color8Bit(0, 0, 255);
    public static final Color8Bit goal6 = new Color8Bit(125, 0, 255);
    public static final Color8Bit goal7 = new Color8Bit(255, 0, 255);
    public static final Color8Bit goal8 = new Color8Bit(255, 128, 84); //this color was handpicked by ava "design lead" grochowski

  }
  
  public static final HashMap<PathPointOpen, Color8Bit> lights = new HashMap<>();
  static {
    lights.put(ScoringPositions.blue0, ScoringLEDs.goal0);
    lights.put(ScoringPositions.blue1, ScoringLEDs.goal1);
    lights.put(ScoringPositions.blue2, ScoringLEDs.goal2);
    lights.put(ScoringPositions.blue3, ScoringLEDs.goal3);
    lights.put(ScoringPositions.blue4, ScoringLEDs.goal4);
    lights.put(ScoringPositions.blue5, ScoringLEDs.goal5);
    lights.put(ScoringPositions.blue6, ScoringLEDs.goal6);
    lights.put(ScoringPositions.blue7, ScoringLEDs.goal7);
    lights.put(ScoringPositions.blue8, ScoringLEDs.goal8);

    lights.put(ScoringPositions.red0, ScoringLEDs.goal0);
    lights.put(ScoringPositions.red1, ScoringLEDs.goal1);
    lights.put(ScoringPositions.red2, ScoringLEDs.goal2);
    lights.put(ScoringPositions.red3, ScoringLEDs.goal3);
    lights.put(ScoringPositions.red4, ScoringLEDs.goal4);
    lights.put(ScoringPositions.red5, ScoringLEDs.goal5);
    lights.put(ScoringPositions.red6, ScoringLEDs.goal6);
    lights.put(ScoringPositions.red7, ScoringLEDs.goal7);
    lights.put(ScoringPositions.red8, ScoringLEDs.goal8);
  }
  public static final PathPointOpen blueSubstation = new PathPointOpen(new Translation2d(0.85, 7.45), Rotation2d.fromDegrees(0));
  public static final PathPointOpen redSubstation = new PathPointOpen(new Translation2d(15.7, 7.45), Rotation2d.fromDegrees(0));
  
  public static final class ElevatorConstants {
    public static final int elevatorMotorID = 25;
    public static final int elevatorFollowerID = 26;
    public static final int elevatorLimitSwitchID = 0;
    // TODO: check this
    public static final double elevatorGearRatio = 5.45;
    public static final ElevatorFeedforward feedforward = new ElevatorFeedforward(1.0e-2, 0.33984/5, 0.01);
    public static final TrapezoidProfile.Constraints elevatorConstraints = new TrapezoidProfile.Constraints(60.0,40.0);
    public static final ProfiledPIDController PIDController = new ProfiledPIDController(0.15/7, 0.0, 0.0139/2, elevatorConstraints);
        static {
          PIDController.setTolerance(
            2.0, //TODO: is this good?
            2.0);
        }

    public static final LinearSystem<N2, N1, N1> elevatorPlant = LinearSystemId.createElevatorSystem(
      DCMotor.getFalcon500(2), 
      Units.lbsToKilograms(7.5), 
      1.75, // Do we need to multiple b/c cascading elevator?
      elevatorGearRatio);

    public static final KalmanFilter<N2, N1, N1> elevatorEstimator = 
      new KalmanFilter<>(
        Nat.N2(),
        Nat.N1(),
        elevatorPlant,
        VecBuilder.fill(2, 40), // How accurate we
        // think our model is, in inches and inches/second.
        VecBuilder.fill(0.001), // How accurate we think our encoder position
        // data is. In this case we very highly trust our encoder position reading.
        0.020);

    public static final LinearQuadraticRegulator<N2, N1, N1> elevatorController = 
        new LinearQuadraticRegulator<>(
          elevatorPlant, VecBuilder.fill(1.0, 10.0), // qelms. Position
          // and velocity error tolerances, in inches and inches per second. Decrease this to more
          // heavily penalize state excursion, or make the controller behave more aggressively.
          VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
          // heavily penalize control effort, or make the controller less aggressive. 12 is a good
          // starting point because that is the (approximate) maximum voltage of a battery.
          0.020);
    
    public static final LinearSystemLoop<N2, N1, N1> elevatorLoop = 
      new LinearSystemLoop<>(
        elevatorPlant, 
        elevatorController, 
        elevatorEstimator, 
        12.0, 
        0.020);

    public static final double elevatorAngleRad = Math.toRadians(41);
    public static final double maxExtensionInches = 49.5;
    public static final Translation2d elevatorOffset = new Translation2d(-5.1, 13.6); // TODO: find actual numbers for this
    // Positions that the end effector needs to be in to score
    // TODO: tune
    public static final Translation2d l1Translation = new Translation2d(20.0, 12.0); 
    //x is distance forward from robot center, y is distance up from floor

    public static final Translation2d l2TranslationCones = new Translation2d(34.0, 34.0);
    public static final Translation2d l3TranslationCones = new Translation2d(50.0, 46.0);
    
    public static final Translation2d l2TranslationCubes = new Translation2d(34.0, 34.0);
    public static final Translation2d l3TranslationCubes = new Translation2d(50.0, 44.0);

    public static final Translation2d humanPlayerTranslation = new Translation2d(30, 37.325); // TODO: Find i dont think the yval is right??

    public static final Translation2d defaultPosition = new Translation2d(5, -5); //TODO: Find

    public static final Constraints elevatorArmSystemConstraints = new Constraints(10.0, 10.0);
    public static final double elevatorSpoolCircumference = 1.751 * Math.PI;
  }

  public static final class ArmConstants {
    public static final int armMotorID = 27;
    public static final int armEncoderID = 1;
    // TODO: Check with actual robot
    public static final double armGearRatio = (12.0 / 18.0) * (1.0 / 45.0) * (1.0 / 1.0);
    public static final ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.0, 0.0);
    public static final TrapezoidProfile.Constraints armConstraints = new TrapezoidProfile.Constraints(3.0,4.0);
    public static final ProfiledPIDController PIDController = new ProfiledPIDController(-1.2, 0.0, 0.0, armConstraints);
    static {
      PIDController.setTolerance(
        0.1, //TODO: is this good?
        0.2);
    }
    public static final double armLengthInches = 12.5;
    public static final double armOffset = 1.1; // encoder native

    public static final double armMinimumAngle = -0.8;
    public static final double armMaximumAngle = -0.05;
  }
  
  /** Constants for simple mechanisms like intake, routing, grabber */
  public static final class MechanismConstants {
    public static final int intakeID = 20;
    public static final int intakeSolenoidForwardID = 3;
    public static final int intakeSolenoidBackwardID = 0;
    public static final double intakeTimeToExtend = 0.1; // TODO: find

    public static final int routingLeftID = 21;
    public static final int routingRightID = 22;
    public static final int routingConveyerID = 23;
    public static final double routingKP = 1; // TODO: tune
    public static final double conveyerKP = 1; // TODO: tune

    public static final int grabberIntakeID = 24;
    public static final int grabberPivotID = 27;
    public static final int grabberSolenoidFrontID = 2;
    public static final int grabberSolenoidBackID = 1;

    public static final int grabberLimitSwitch = 3;
    public static final int grabberBeambreak = 2;

    // In falcon native units, before reduction because i was too lazy to do conversions
    public static final double grabberScoringRotation = 3.0e4;
    public static final double grabberSingleSubstationRotation = 1e3;
    public static final double grabberDoubleSubstationRotation = 2.5e4;
    public static final double grabberRoutingRotation = 2e3;
    public static final double grabberStoringRotation = 1e3;
  }

  public static final class LEDConstants {
    public static final int ledPort = 1;
    public static final int ledLength = 140; //TODO: find

    public static final Color8Bit defaultColor = new Color8Bit(58 / 2, 11 / 2, 110 / 2);
  }

  public static final class ScoringLevels {
    public static final double topConeLevel = 48; //this is in inches
    public static final double topCubeLevel = 48;
    public static final double midConeLevel = 30.5;
    public static final double midCubeLevel = 29;
    public static final double bottomLevel = 20;

    public static final double chuteLevel = 15.0;
  }

  public static final double humanPlayerLevel = 40.5; //or substation idk what we're calling them

  /** For 5026 vision code */
  public static final class Vision {
    public static class VisionSource {
      public String name;
      public Transform3d robotToCamera;
      public VisionSource (String name, Transform3d robotToCamera) {
        this.name = name;
        this.robotToCamera = robotToCamera;
      }
    }

    public static final List<VisionSource> VISION_SOURCES =
        List.of(
            // new VisionSource(
            //     "limelight-right",
            //     new Transform3d(
            //       new Translation3d(
            //         Units.inchesToMeters(-8),
            //         Units.inchesToMeters(-9.75), 
            //         Units.inchesToMeters(-22.75)),
            //       new Rotation3d(0, 0, Units.degreesToRadians(-5)))),
            new VisionSource(
                "limelight-left",
                new Transform3d(
                  new Translation3d(
                    Units.inchesToMeters(-8),
                    Units.inchesToMeters(9.75), 
                    Units.inchesToMeters(-22.75)),
                  new Rotation3d(0, 0, Units.degreesToRadians(5)))));
  }

  /** For 5026 vision code */
  public static final class PoseEstimator {
    /**
     * Standard deviations of model states. Increase these numbers to trust your model's state
     * estimates less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS =
        Matrix.mat(Nat.N3(), Nat.N1())
            .fill(
                0.1, // x
                0.1, // y
                0.1 // theta
                );

    /**
     * Standard deviations of the vision measurements. Increase these numbers to trust global
     * measurements from vision less. This matrix is in the form [x, y, theta]ᵀ, with units in
     * meters and radians.
     */
    public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS =
        Matrix.mat(Nat.N3(), Nat.N1())
            .fill(
                // if these numbers are less than one, multiplying will do bad things
                1, // x
                1, // y
                1 * Math.PI // theta
                );

    /** The distance at which tag distance is factored into deviation */
    public static final double NOISY_DISTANCE_METERS = 1.5;

    /**
     * The number to multiply by the smallest of the distance minus the above constant, clamped
     * above 1 to be the numerator of the fraction.
     */
    public static final double DISTANCE_WEIGHT = 7;

    /**
     * The number to multiply by the number of tags beyond the first to get the denominator of the
     * deviations matrix.
     */
    public static final double TAG_PRESENCE_WEIGHT = 10;

    /** The amount to shift the pose ambiguity by before multiplying it. */
    public static final double POSE_AMBIGUITY_SHIFTER = .2;

    /** The amount to multiply the pose ambiguity by if there is only one tag. */
    public static final double POSE_AMBIGUITY_MULTIPLIER = 4;

    /** about one inch */
    public static final double DRIVE_TO_POSE_XY_ERROR_MARGIN_METERS = .05;

    public static final double DRIVE_TO_POSE_THETA_ERROR_MARGIN_DEGREES = 2;
  }

}