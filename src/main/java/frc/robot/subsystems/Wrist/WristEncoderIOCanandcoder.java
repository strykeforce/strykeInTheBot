package frc.robot.subsystems.Wrist;

import com.reduxrobotics.sensors.canandcoder.CANandcoder;
import frc.robot.constants.WristConstants;
import java.util.Set;
import org.strykeforce.telemetry.Registrable;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.Measurable;
import org.strykeforce.telemetry.measurable.Measure;

public class WristEncoderIOCanandcoder implements WristEncoderIO, Measurable, Registrable {

  private CANandcoder encoder;

  public WristEncoderIOCanandcoder() {
    encoder = new CANandcoder(WristConstants.kWristEncoderId);
  }

  public void updateInputs(WristEncoderIOInputs inputs) {
    inputs.absolutePercentage = encoder.getAbsPosition();
  }

  @Override
  public void setSelectedSensorPos(double absolutePercentage) {}

  // ------------ grapher stuff -------------------

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(this);
  }

  @Override
  public int getDeviceId() {
    return WristConstants.kWristEncoderId;
  }

  @Override
  public String getDescription() {
    return "CANAndCoder";
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("Absolute percentage", () -> encoder.getAbsPosition()),
        new Measure("relative percentage", () -> encoder.getPosition()),
        new Measure("velocity", () -> encoder.getVelocity()));
  }

  @Override
  public String getType() {
    return "CANAndCoder";
  }

  @Override
  public int compareTo(Measurable item) {
    return 0;
  }
}
