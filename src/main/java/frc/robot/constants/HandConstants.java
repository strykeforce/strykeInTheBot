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
}
