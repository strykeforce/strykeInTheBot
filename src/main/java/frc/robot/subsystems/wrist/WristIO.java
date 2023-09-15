package frc.robot.subsystems.wrist;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface WristIO {

  @AutoLog
  public static class WristIOInputs {
    public double positionTicks = 0.0;
    public double absoluteTicks = 0.0;
    public double velocityTicksPer100ms = 0.0;
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setSelectedSensorPos(double positionTicks) {}

  public default void setPos(double positionTicks) {}

  public default void setPct(double percentOutput) {}

  public default void setSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {}

  public default void registerWith(TelemetryService telemetryService) {}
}
