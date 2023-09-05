package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

/** Automatically generated file containing build version information. */
public final class ShoulderConstants {

  public static int kLeftMainID = 20;
  public static int kLeftFollowID = 21;
  public static int kRightFollow1ID = 22;
  public static int kRightFollow2ID = 22;

  public static double kOpenPos;

  public static double kHoldPos;

  public static double kZeroTicks;

  public static final double kShoulderMaxFwd = 2_000;
  public static final double kShoulderMaxRev = -1_000;
  public static final double kShoulderZeroTicks = 550;

  public static final double kShoulderHoldPos = 500;
  public static final double kShoulderOpenPos = 1_800;
  public static final double kShoulderCloseEnough = 100;

  public static final double kShoulderGearRatio = 1;

  public static final double kCloseEnough = 0;

  public static TalonSRXConfiguration getShoulderTalonConfig() {

    TalonSRXConfiguration shoulderConfig = new TalonSRXConfiguration();

    // PID Configs
    shoulderConfig.slot0.kP = 2.0;
    shoulderConfig.slot0.kI = 0.0;
    shoulderConfig.slot0.kD = 50.0;
    shoulderConfig.slot0.kF = 0.85;
    shoulderConfig.slot0.integralZone = 0;
    shoulderConfig.slot0.allowableClosedloopError = 40;
    shoulderConfig.motionCruiseVelocity = 1_000;
    shoulderConfig.motionAcceleration = 10_000;

    // Soft Limits
    shoulderConfig.forwardSoftLimitEnable = true;
    shoulderConfig.forwardSoftLimitThreshold = kShoulderMaxFwd;
    shoulderConfig.reverseSoftLimitEnable = true;
    shoulderConfig.reverseSoftLimitThreshold = kShoulderMaxRev;

    // General Talon
    shoulderConfig.neutralDeadband = 0.04;
    shoulderConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
    shoulderConfig.velocityMeasurementWindow = 64;
    shoulderConfig.voltageCompSaturation = 12;
    shoulderConfig.voltageMeasurementFilter = 32;

    return shoulderConfig;
  }

  public static SupplyCurrentLimitConfiguration getShoulderSupplyLimitConfig() {
    return new SupplyCurrentLimitConfiguration(true, 2, 2, 0.3);
  }
}
