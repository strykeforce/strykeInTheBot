package frc.robot.subsystems.shoulder;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface MinimalShoulderIO {

  @AutoLog
  public static class ShoulderIOInputs {
    public double positionTicks = 0.0;
    public double absoluteTicks = 0.0;
    public double velocityTicksPer100ms = 0.0;
    public boolean isFwdLimitSwitchClosed = false;
  }

  public default void setSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {}

  public default void registerWith(TelemetryService telemetryService) {}

  public default void updateInputs(ShoulderIOInputs inputs) {}

  public default void setSelectedSensorPos(double positionTicks) {}

  public default void setPos(double positionTicks) {}

  public default void setPct(double percentOutput) {}

  public default void disableOutput() {}

public void configSoftLimitEnable(boolean b);

public void configStatorCurrentLimit(StatorCurrentLimitConfiguration elevStatorCurrentLimitConfiguration);

public double getSelectedSensorPosition();

public void configSupplyCurrentLimit(SupplyCurrentLimitConfiguration shoulderSupplyLimitConfig,
        int ktalonconfigtimeout);
}
