package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

public class WristConstants {
  public static int kWristTalonMainId = 30;
  public static int kWristTalonFollowId = 31;

  public static final double kMaxFwd = 2_000;
  public static final double kMaxRev = -1_000;
  public static final double kZeroTicks = 550;
  public static final double kZeroSpeed = -1;
  public static final double kVelocityThreshhold = 0.01;
  public static final int kZeroCount = 3;

  public static final double kHoldPos = 500;
  public static final double kOpenPos = 1_800;
  public static final double kCloseEnough = 100;

  public static TalonFXConfiguration getWristTalonConfig() {
    TalonFXConfiguration WristConfig = new TalonFXConfiguration();

    // PID Configs
    WristConfig.slot0.kP = 2.0;
    WristConfig.slot0.kI = 0.0;
    WristConfig.slot0.kD = 50.0;
    WristConfig.slot0.kF = 0.85;
    WristConfig.slot0.integralZone = 0;
    WristConfig.slot0.allowableClosedloopError = 40;
    WristConfig.motionCruiseVelocity = 1_000;
    WristConfig.motionAcceleration = 10_000;

    // Soft Limits
    WristConfig.forwardSoftLimitEnable = true;
    WristConfig.forwardSoftLimitThreshold = kMaxFwd;
    WristConfig.reverseSoftLimitEnable = true;
    WristConfig.reverseSoftLimitThreshold = kMaxRev;

    // General Talon
    WristConfig.neutralDeadband = 0.04;
    WristConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
    WristConfig.velocityMeasurementWindow = 64;
    WristConfig.voltageCompSaturation = 12;
    WristConfig.voltageMeasurementFilter = 32;

    return WristConfig;
  }

  public static SupplyCurrentLimitConfiguration getWristSupplyLimitConfig() {
    return new SupplyCurrentLimitConfiguration(true, 2, 2, 0.3);
  }

  public static SupplyCurrentLimitConfiguration getWristSupplyLimitZeroingConfig() {
    return new SupplyCurrentLimitConfiguration(true, 0.5, 2, 0.3);
  }
}