package frc.robot.subsystems.Extendo;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface ExtendoIO {

  @AutoLog
  public static class ExtendoIOInputs {
    public double positionTicks = 0.0;
    public double absoluteTicks = 0.0;
    public double velocityTicksPer100ms = 0.0;
  }

  public default void updateInputs(ExtendoIOInputs inputs) {}

  public default void setSelectedSensorPos(double positionTicks) {}

  public default void setPos(double positionTicks) {}

  public default void setPct(double percentOutput) {}

  public default void setSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {}

  public default void registerWith(TelemetryService telemetryService) {}
}
