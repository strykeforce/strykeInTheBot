package frc.robot.subsystems.example;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface ExampleIO {

  @AutoLog
  public static class ExampleIOInputs {
    public double positionTicks = 0.0;
    public double absoluteTicks = 0.0;
    public double velocityTicksPer100ms = 0.0;
    public boolean isFwdLimitSwitchClosed = false;
  }

  public default void updateInputs(ExampleIOInputs inputs) {}

  public default void setSelectedSensorPos(double positionTicks) {}

  public default void setPos(double positionTicks) {}

  public default void setPct(double percentOutput) {}

  public default void setSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {}

  public default void registerWith(TelemetryService telemetryService) {}
}
