package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

public final class MinimalShoulderConstants {
  public static int kLeftMainID = 20;

  public static double kOpenPos;

  public static double kHoldPos;

  public static double kZeroTicks;

  // Shoulder scoring
  public static final double kShoulderCubeL1 = -2_000;
  public static final double kShoulderCubeL2 = -78_000; // -14_929
  public static final double kShoulderCubeL3 = -73_000; // -25_000

  public static final double kShoulderConeL1 = -2_000;
  public static final double kShoulderConeL2 = -13_725;

  public static final double kShoulderMaxFwd = -1_000;
  public static final double kShoulderMaxRev = -120_000;
  public static final double kShoulderZeroTicks = 550;

  public static final double kShoulderHoldPos = 500;
  public static final double kShoulderOpenPos = 1_800;

  public static final double kShoulderGearRatio = 1;

  public static final double kCloseEnough = 640;

  public static final int kZeroTargetSpeedTicksPer100ms = 5; // FIXME

  public static final int kZeroStableCounts = 2; // FIXME

  public static final int kTalonConfigTimeout = 10; // FIXME

  public static final double kZeroSpeed = 0.3; // FIXME

  public static final double kStowShoulderPos = -2_000;

  public static final double kShelfCubeShoulderPos =
      -50_785; // OLD: -9_949 (fwd pickup, not backwards)

  public static final double kShelfConeShoulderPos = -9_949; // Unused

  public static final double kFloorPickupCube = -120_000;

  public static final double kFloorPickupCone = -100_000;

  public static TalonFXConfiguration getShoulderFalconConfig() {
    TalonFXConfiguration falconConfig = new TalonFXConfiguration();

    // PID Configs
    falconConfig.slot0.kP = 0.3;
    falconConfig.slot0.kI = 0;
    falconConfig.slot0.kD = 0;
    falconConfig.slot0.kF = 0.05;
    falconConfig.slot0.integralZone = 0;
    falconConfig.slot0.maxIntegralAccumulator = 0;
    falconConfig.slot0.allowableClosedloopError = 10;
    falconConfig.motionCruiseVelocity = 10_000;
    falconConfig.motionAcceleration = 20_000;

    // Soft Limits
    falconConfig.forwardSoftLimitEnable = true;
    falconConfig.forwardSoftLimitThreshold = kShoulderMaxFwd;
    falconConfig.reverseSoftLimitEnable = true;
    falconConfig.reverseSoftLimitThreshold = kShoulderMaxRev;

    // General Talon
    falconConfig.neutralDeadband = 0.01;
    falconConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
    falconConfig.velocityMeasurementWindow = 64;
    falconConfig.voltageCompSaturation = 12;
    falconConfig.voltageMeasurementFilter = 32;

    return falconConfig;
  }

  public static StatorCurrentLimitConfiguration getShoulderStatorTurnOff() {
    return new StatorCurrentLimitConfiguration(false, 0.0, 0.0, 0.0);
  }

  public static StatorCurrentLimitConfiguration getShoulderZeroStatorLimitConfig() {
    return new StatorCurrentLimitConfiguration(true, 4, 4, 0.001);
  }

  public static SupplyCurrentLimitConfiguration getShoulderSupplyLimitConfig() {
    return new SupplyCurrentLimitConfiguration(true, 15, 15, 0);
  }
}
