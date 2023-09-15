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
  public static final double kShoulderCubeL1 = 0;
  public static final double kShoulderCubeL2 = 0;
  public static final double kShoulderCubeL3 = 0;

  public static final double kShoulderConeL1 = 0;
  public static final double kShoulderConeL2 = 0;

  public static final double kShoulderMaxFwd = 2_000;
  public static final double kShoulderMaxRev = -1_000;
  public static final double kShoulderZeroTicks = 550;

  public static final double kShoulderHoldPos = 500;
  public static final double kShoulderOpenPos = 1_800;
  public static final double kShoulderCloseEnough = 100;

  public static final double kShoulderGearRatio = 1;

  public static final double kCloseEnough = 0; // FIXME

  public static final int kZeroTargetSpeedTicksPer100ms = 10; // FIXME

  public static final int kZeroStableCounts = 10; // FIXME

  public static final int kTalonConfigTimeout = 10; // FIXME

  public static final double kZeroSpeed = 0; // FIXME

  public static final double kStowShoulderPos = 0; // FIXME

  public static final double kShelfCubeShoulderPos = 0;

  public static final double kShelfConeShoulderPos = 0;

  public static final double kFloorPickupCube = 0;

  public static final double kFloorPickupCone = 0;

  public static StatorCurrentLimitConfiguration getElevStatorCurrentLimitConfiguration() {
    return new StatorCurrentLimitConfiguration(true, 8.0, 8.0, 0.001);
  }

  public static TalonFXConfiguration getShoulderFalconConfig() {
    TalonFXConfiguration falconConfig = new TalonFXConfiguration();

    // PID Configs
    falconConfig.slot0.kP = 2.0;
    falconConfig.slot0.kI = 0.0;
    falconConfig.slot0.kD = 50.0;
    falconConfig.slot0.kF = 0.85;
    falconConfig.slot0.integralZone = 0;
    falconConfig.slot0.allowableClosedloopError = 40;
    falconConfig.motionCruiseVelocity = 1_000;
    falconConfig.motionAcceleration = 10_000;

    // Soft Limits
    falconConfig.forwardSoftLimitEnable = true;
    falconConfig.forwardSoftLimitThreshold = kShoulderMaxFwd;
    falconConfig.reverseSoftLimitEnable = true;
    falconConfig.reverseSoftLimitThreshold = kShoulderMaxRev;

    // General Talon
    falconConfig.neutralDeadband = 0.04;
    falconConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
    falconConfig.velocityMeasurementWindow = 64;
    falconConfig.voltageCompSaturation = 12;
    falconConfig.voltageMeasurementFilter = 32;

    return falconConfig;
  }

  public static StatorCurrentLimitConfiguration getShoulderStatorTurnOff() {
    return new StatorCurrentLimitConfiguration(false, 0.0, 0.0, 0.0);
  }

  public static SupplyCurrentLimitConfiguration getShoulderSupplyLimitConfig() {
    return new SupplyCurrentLimitConfiguration(true, 2, 2, 0.3);
  }
}
