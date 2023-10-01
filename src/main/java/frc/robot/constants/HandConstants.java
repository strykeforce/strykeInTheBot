package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

public class HandConstants {
  public static int kHandFalconID = 50;

  public static TalonFXConfiguration getHandFalconConfig() {
    TalonFXConfiguration falconConfig = new TalonFXConfiguration();

    falconConfig.neutralDeadband = 0.01;
    falconConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
    falconConfig.velocityMeasurementWindow = 64;
    falconConfig.voltageCompSaturation = 12;
    falconConfig.voltageMeasurementFilter = 32;
    falconConfig.supplyCurrLimit.currentLimit = 15;
    falconConfig.supplyCurrLimit.triggerThresholdCurrent = 15;
    falconConfig.supplyCurrLimit.triggerThresholdTime = 0;
    falconConfig.supplyCurrLimit.enable = false;

    falconConfig.statorCurrLimit.currentLimit = 30;
    falconConfig.statorCurrLimit.triggerThresholdCurrent = 30;
    falconConfig.statorCurrLimit.triggerThresholdTime = 0.1;
    falconConfig.statorCurrLimit.enable = true;

    return falconConfig;
  }

  public static final double kWaitingSubstationSpeed = 0.3; // used comp robot speed
  public static final double kWaitingFloorSpeed = 0.6;
  public static final double kConeSpeed = 0.05; // used comp robot speed
  public static final double kCubeSpeed = 0.05; // used comp robot speed

  public static final double kConeVelLimit = 50; // used comp robot number

  public static final int kHasConeStableCounts = 10; // used comp robot number
  public static final int kHasCubeStableCounts = 15; // used comp robot number
  public static final int kEjectStableCounts = 5; // FIXME guessed at number

  public static final double kHandEjectConeSpeedL1 = -0.2;
  public static final double kHandEjectConeSpeedL2 = 0;
  public static final double kHandEjectConeSpeedL3 = 0;

  public static final double kHandEjectCubeSpeedL1 = -0.15;
  public static final double kHandEjectCubeSpeedL2 = -1;
  public static final double kHandEjectCubeSpeedL3 = -1;

  public static final int kHandEjectConeCountsL1 = 20;
  public static final int kHandEjectConeCountsL2 = 20;
  public static final int kHandEjectConeCountsL3 = 20;

  public static final int kHandEjectCubeCountsL1 = 20;
  public static final int kHandEjectCubeCountsL2 = 20;
  public static final int kHandEjectCubeCountsL3 = 20;
}
