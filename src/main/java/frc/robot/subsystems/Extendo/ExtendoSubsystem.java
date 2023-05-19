package frc.robot.subsystems.Extendo;

import frc.robot.constants.ExtendoConstants;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ExtendoSubsystem extends MeasurableSubsystem {
  private final ExtendoIO io;
  private final ExtendoIOInputsAutoLogged inputs = new ExtendoIOInputsAutoLogged();
  private double setpointTicks;
  private Logger logger = LoggerFactory.getLogger(ExtendoSubsystem.class);
  private org.littletonrobotics.junction.Logger advLogger =
      org.littletonrobotics.junction.Logger.getInstance();

  public ExtendoSubsystem(ExtendoIO io) {
    this.io = io;
  }

  public void zero() {
    double absolute = inputs.absoluteTicks;
    
    double offset = absolute - ExtendoConstants.kZeroTicks;
    io.setSelectedSensorPos(offset);
    logger.info("Abs: {}, Zero Pos: {}, Offset: {}", absolute, ExtendoConstants.kZeroTicks, offset);
  }

  public boolean isFinished() {
    return true;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    advLogger.processInputs("Extendo", inputs);
    

    // Log Outputs
    advLogger.recordOutput("Extendo/setpointTicks", setpointTicks);
  }

  // Grapher Stuff
  @Override
  public Set<Measure> getMeasures() {
    return Set.of();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    io.registerWith(telemetryService);
  }
}
