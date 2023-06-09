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
  private final WristEncoderIO eio;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private double setpointTicks;
  private Logger logger = LoggerFactory.getLogger(WristSubsystem.class);
  private org.littletonrobotics.junction.Logger advLogger =
      org.littletonrobotics.junction.Logger.getInstance();
  private ZeroState zeroState = ZeroState.IDLE;
  private int ZeroCounter = 0;
  private double SetPoint = 0;

  public WristSubsystem(WristIO io, WristEncoderIO eio) {
    this.io = io;
    this.eio = eio;
  }

  public void zero() {
    // double absfalcon = inputs.absoluteTicks;
    // double absCANAnd = eio.inputs.absolutePercentage;
    // double offset = absolute - WristConstants.kZeroTicks;
    // io.setSelectedSensorPos(offset);
    // logger.info("Abs: {}, Zero Pos: {}, Offset: {}", absfalcon, WristConstants.kZeroTicks, offset);
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

    if (zeroState == ZeroState.ZEROING) {
      if (inputs.velocityTicksPer100ms <= WristConstants.kVelocityThreshhold) {
        ZeroCounter++;
        if (ZeroCounter >= WristConstants.kZeroCount) {
          double absolute = inputs.absoluteTicks;
          double offset = absolute - WristConstants.kZeroTicks;
          io.setSelectedSensorPos(offset);
          logger.info(
              "Abs: {}, Zero Pos: {}, Offset: {}", absolute, WristConstants.kZeroTicks, offset);
          this.io.setPct(0);
          zeroState = ZeroState.IDLE;
          ZeroCounter = 0;
          io.setSupplyCurrentLimit(WristConstants.getWristSupplyLimitConfig());
        }
      } else {
        ZeroCounter = 0;
      }
    }

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

  public enum ZeroState {
    IDLE,
    ZEROING;
  }
}
