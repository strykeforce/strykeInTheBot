package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

public class ExtendoConstants {
  public static int kExtendoTalonMainId = 30;
  public static int kExtendoTalonFollowId = 31;

  public static final double kMaxFwd = 2_000;
  public static final double kMaxRev = -1_000;
  public static final double kZeroTicks = 550;
  public static final double kZeroSpeed = -1;
  public static final double kVelocityThreshhold = 0.01;
  public static final int kZeroCount = 3;

  public static final double kHoldPos = 500;
  public static final double kOpenPos = 1_800;
  public static final double kCloseEnough = 100;

  public static TalonFXConfiguration getExtendoTalonConfig() {
    TalonFXConfiguration ExtendoConfig = new TalonFXConfiguration();

    // PID Configs
    ExtendoConfig.slot0.kP = 2.0;
    ExtendoConfig.slot0.kI = 0.0;
    ExtendoConfig.slot0.kD = 50.0;
    ExtendoConfig.slot0.kF = 0.85;
    ExtendoConfig.slot0.integralZone = 0;
    ExtendoConfig.slot0.allowableClosedloopError = 40;
    ExtendoConfig.motionCruiseVelocity = 1_000;
    ExtendoConfig.motionAcceleration = 10_000;

    // Soft Limits
    ExtendoConfig.forwardSoftLimitEnable = true;
    ExtendoConfig.forwardSoftLimitThreshold = kMaxFwd;
    ExtendoConfig.reverseSoftLimitEnable = true;
    ExtendoConfig.reverseSoftLimitThreshold = kMaxRev;

    // General Talon
    ExtendoConfig.neutralDeadband = 0.04;
    ExtendoConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
    ExtendoConfig.velocityMeasurementWindow = 64;
    ExtendoConfig.voltageCompSaturation = 12;
    ExtendoConfig.voltageMeasurementFilter = 32;

    return ExtendoConfig;
  }

  public static SupplyCurrentLimitConfiguration getExtendoSupplyLimitConfig() {
    return new SupplyCurrentLimitConfiguration(true, 2, 2, 0.3);
  }

  public static SupplyCurrentLimitConfiguration getExtendoSupplyLimitZeroingConfig() {
    return new SupplyCurrentLimitConfiguration(true, 0.5, 2, 0.3);
  }
}
