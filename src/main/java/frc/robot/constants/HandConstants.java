package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

public class HandConstants {
  public static int kHandFalconID = 50;

  public static TalonFXConfiguration getHandFalconConfig() {
    TalonFXConfiguration falconConfig = new TalonFXConfiguration();

    falconConfig.neutralDeadband = 0.04;
    falconConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
    falconConfig.velocityMeasurementWindow = 64;
    falconConfig.voltageCompSaturation = 12;
    falconConfig.voltageMeasurementFilter = 32;
    falconConfig.supplyCurrLimit.currentLimit = 20;
    falconConfig.supplyCurrLimit.triggerThresholdCurrent = 45;
    falconConfig.supplyCurrLimit.triggerThresholdTime = 0.04;
    falconConfig.supplyCurrLimit.enable = false;

    falconConfig.statorCurrLimit.currentLimit = 30;
    falconConfig.statorCurrLimit.triggerThresholdCurrent = 30;
    falconConfig.statorCurrLimit.triggerThresholdTime = 0.1;
    falconConfig.statorCurrLimit.enable = true;

    return falconConfig;
  }

  public static final double kWaitingSpeed = 0.7; // used comp robot speed
  public static final double kConeSpeed = 0.2; // used comp robot speed
  public static final double kCubeSpeed = 0.15; // used comp robot speed

  public static final double kConeVelLimit = 50; // used comp robot number

  public static final int kHasConeStableCounts = 2; // used comp robot number
  public static final int kHasCubeStableCounts = 2; // used comp robot number
  public static final int kEjectStableCounts = 5; // FIXME guessed at number
}
