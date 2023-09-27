package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface WristEncoderIO {

  @AutoLog
  public static class WristEncoderIOInputs {
    public double absolutePercentage = 0.0;
  }

  public default void updateInputs(WristEncoderIOInputs inputs) {}

  public default void registerWith(TelemetryService telemetryService) {}
}
