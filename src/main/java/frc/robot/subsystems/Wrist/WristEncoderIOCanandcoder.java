package frc.robot.subsystems.Wrist;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

import com.reduxrobotics.sensors.canandcoder.CANandcoder;

import frc.robot.constants.WristConstants;

public class WristEncoderIOCanandcoder implements WristEncoderIO{

  private CANandcoder encoder;

  @AutoLog
  public static class WristIOEncoderInputs {
    public double absolutePercentage = 0.0;
  }

  @Override
  public void updateInputs(WristEncoderIOInputs inputs) {
    inputs.absolutePercentage = encoder.getAbsPosition();
  }

  @Override
  public void setSelectedSensorPos(double absolutePercentage) {}

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(encoder);}
}
