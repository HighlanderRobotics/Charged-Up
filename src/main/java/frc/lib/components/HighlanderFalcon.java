// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.components;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/** A talonfx wrapper. Lazy talon functionality is taken from 254 */
public class HighlanderFalcon extends TalonFX {
  private double lastSet = 0.0;
  private TalonFXControlMode lastControlMode = null;

  static final double TICKS_TO_ROTATIONS = 1.0 / 2048.0;
  static final double ROTATIONS_TO_TICKS = 2048;

  private double gearRatio = 1.0;

  public HighlanderFalcon(int id) {
    super(id);
    bandWithLimitMotorCAN(this);
  }

  public HighlanderFalcon(int id, double gearing) {
    super(id);
    gearRatio = gearing;
    bandWithLimitMotorCAN(this);
  }

  public HighlanderFalcon(int id, String canbus, double gearing) {
    super(id, canbus);
    gearRatio = gearing;
    bandWithLimitMotorCAN(this);
  }

  public HighlanderFalcon(int id, String canbus) {
    super(id, canbus);
    bandWithLimitMotorCAN(this);
  }

  /** Makes a new HighlanderFalcon with a PID controller built in */
  public HighlanderFalcon(int id, double gearing, double p, double i, double d) {
    super(id);
    gearRatio = gearing;
    this.config_kP(0, p);
    this.config_kI(0, i);
    this.config_kD(0, d);
    bandWithLimitMotorCAN(this);
  }

  /** Makes a new HighlanderFalcon with a PID controller built in */
  public HighlanderFalcon(int id, String canbus, double gearing, double p, double i, double d) {
    super(id, canbus);
    gearRatio = gearing;
    this.config_kP(0, p);
    this.config_kI(0, i);
    this.config_kD(0, d);
    bandWithLimitMotorCAN(this);
  }

  public HighlanderFalcon(
      int id,
      double gearing,
      double p,
      double i,
      double d,
      double accelRpmSquared,
      double maxVelRpm) {
    super(id);
    gearRatio = gearing;
    this.config_kP(0, p);
    this.config_kI(0, i);
    this.config_kD(0, d);
    this.configMotionAcceleration(HighlanderFalcon.rotToNative(accelRpmSquared));
    this.configMotionCruiseVelocity(HighlanderFalcon.rpmToNative(maxVelRpm));
    bandWithLimitMotorCAN(this);
  }

  public void defaultConfigs() {
    this.enableVoltageCompensation(true);
    this.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 80, 0.5));
  }

  public double getGearing() {
    return gearRatio;
  }

  /** Gets the last value sent to the motor */
  public double getLastSet() {
    return lastSet;
  }

  /** Sets the motors behaviour if the new value is different then the last one. */
  @Override
  public void set(TalonFXControlMode mode, double value) {
    if (value != lastSet || mode != lastControlMode) {
      lastSet = value;
      lastControlMode = mode;
      super.set(mode, value);
    }
  }

  /** Gets the encoder position in rotations */
  public double getRotations() {
    return getSelectedSensorPosition() * TICKS_TO_ROTATIONS * (1.0 / gearRatio);
  }

  /** Gets the encoder position in degrees */
  public double getDegrees() {
    return getRotations() * 360;
  }

  /** Gets the encoder position in radians */
  public double getRadians() {
    return getRotations() * Math.PI * 2;
  }

  /** Gets the encoder RPM */
  public double getRPM() {
    return this.getSelectedSensorVelocity() * 10 * 60 * (1 / gearRatio);
  }

  /** Gets the encoder rotations per second */
  public double getRPS() {
    return getRPM() / 60;
  }

  /** Sets the onboard velocity PID */
  public void setTargetRPM(double rpm) {
    set(TalonFXControlMode.Velocity, rpm * ROTATIONS_TO_TICKS * 60 * 10 * gearRatio);
  }

  /** Sets the onboard position PID, in rotations */
  public void setTargetRot(double rotations) {
    set(TalonFXControlMode.Position, rotations * ROTATIONS_TO_TICKS * gearRatio);
  }

  /** Sets the onboard position PID, in degrees */
  public void setTargetDegrees(double degrees) {
    setTargetRot((degrees / 360) * gearRatio);
  }

  /** Sets the onboard position PID, in radians */
  public void setTargetRadians(double radians) {
    setTargetRot((radians / (Math.PI * 2)) * gearRatio);
  }

  /** Sets the output of the motor from -1 to 1 */
  public void setPercentOut(double percent) {
    set(TalonFXControlMode.PercentOutput, percent);
  }

  public void setMotionMagic(double pos) {
    set(TalonFXControlMode.MotionMagic, pos);
  }

  public static int rotToNative(double rot) {
    return (int) (rot * 2048.0);
  }

  public static int radToNative(double rad) {
    return rotToNative(rad / (Math.PI * 2));
  }

  public static int rpmToNative(double rpm) {
    return (int) rotToNative(rpm / 600);
  }

  /** Stolen from 841. Limits CAN network usage */
  private static void bandWithLimitMotorCAN(TalonFX motor) {
    motor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
    motor.setStatusFramePeriod(StatusFrame.Status_1_General, 40);
    motor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 255);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
    motor.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255);
    motor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 255);
    motor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 255);
    motor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 255);
    motor.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 255);
  }
}
