package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

public class ExampleConstants {
  public static int kExampleTalonId = 5;

  public static final double kMaxFwd = 2_000;
  public static final double kMaxRev = -1_000;
  public static final double kZeroTicks = 550;

  public static final double kHoldPos = 500;
  public static final double kOpenPos = 1_800;
  public static final double kCloseEnough = 100;

  public static TalonSRXConfiguration getExampleTalonConfig() {
    TalonSRXConfiguration exampleConfig = new TalonSRXConfiguration();

    // PID Configs
    exampleConfig.slot0.kP = 2.0;
    exampleConfig.slot0.kI = 0.0;
    exampleConfig.slot0.kD = 50.0;
    exampleConfig.slot0.kF = 0.85;
    exampleConfig.slot0.integralZone = 0;
    exampleConfig.slot0.allowableClosedloopError = 40;
    exampleConfig.motionCruiseVelocity = 1_000;
    exampleConfig.motionAcceleration = 10_000;

    // Soft Limits
    exampleConfig.forwardSoftLimitEnable = true;
    exampleConfig.forwardSoftLimitThreshold = kMaxFwd;
    exampleConfig.reverseSoftLimitEnable = true;
    exampleConfig.reverseSoftLimitThreshold = kMaxRev;

    // General Talon
    exampleConfig.neutralDeadband = 0.04;
    exampleConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
    exampleConfig.velocityMeasurementWindow = 64;
    exampleConfig.voltageCompSaturation = 12;
    exampleConfig.voltageMeasurementFilter = 32;

    return exampleConfig;
  }

  public static SupplyCurrentLimitConfiguration getExampleSupplyLimitConfig() {
    return new SupplyCurrentLimitConfiguration(true, 2, 2, 0.3);
  }
}
