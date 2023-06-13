package frc.robot.subsystems.Wrist;

import frc.robot.constants.WristConstants;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class WristSubsystem extends MeasurableSubsystem {
  private final WristIO io;
  private final WristEncoderIO Eio;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private final WristEncoderIOInputsAutoLogged Einputs = new WristEncoderIOInputsAutoLogged();
  private double setpointTicks;
  private Logger logger = LoggerFactory.getLogger(WristSubsystem.class);
  private org.littletonrobotics.junction.Logger advLogger =
      org.littletonrobotics.junction.Logger.getInstance();
  private double SetPoint = 0;

  public WristSubsystem(WristIO io, WristEncoderIO Eio) {
    this.io = io;
    this.Eio = Eio;
  }

  public void zero() {
    double absTalon = inputs.absoluteTicks;
    double absCANAnd = Einputs.absolutePercentage * WristConstants.kTalonTicks;
    double offset = absCANAnd - WristConstants.kZeroTicks;
    io.setSelectedSensorPos(offset);
    logger.info(
        "AbsTalon: {}, AbsCANAnd: {}, Zero Pos: {}, Offset: {}",
        absTalon,
        absCANAnd,
        WristConstants.kZeroTicks,
        offset);
  }

  public boolean isFinished() {
    return (WristConstants.kCloseEnough <= Math.abs(inputs.positionTicks - SetPoint));
  }

  public boolean isPastPoint(double pastPointTicks) {

    //  Tests if the pos is between the input and the SetPoint.
    //  This is the best way I could find.

    if (SetPoint < inputs.positionTicks) {
      return (pastPointTicks + WristConstants.kCloseEnough >= inputs.positionTicks);
    } else if (SetPoint > inputs.positionTicks) {
      return (pastPointTicks - WristConstants.kCloseEnough <= inputs.positionTicks);
    } else {
      return false;
    }
  }

  public void setPos(int positionTicks) {
    this.io.setPos(positionTicks);
    SetPoint = positionTicks;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    advLogger.processInputs("Wrist", inputs);

    // Log Outputs
    advLogger.recordOutput("Wrist/setpointTicks", setpointTicks);
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
