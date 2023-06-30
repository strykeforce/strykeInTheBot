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
  private double setpointTicks = 0;
  private Logger logger = LoggerFactory.getLogger(ExtendoSubsystem.class);
  private org.littletonrobotics.junction.Logger advLogger = org.littletonrobotics.junction.Logger.getInstance();
  private ZeroState zeroState = ZeroState.IDLE;
  private int ZeroCounter = 0;

  public ExtendoSubsystem(ExtendoIO io) {
    this.io = io;
  }

  public void zero() {
    io.setSupplyCurrentLimit(ExtendoConstants.getExtendoSupplyLimitZeroingConfig());
    this.io.setPct(ExtendoConstants.kZeroSpeed);
    zeroState = ZeroState.ZEROING;
  }

  public boolean isFinished() {
    return (ExtendoConstants.kCloseEnough >= Math.abs(inputs.positionTicks-setpointTicks));
  }

  public boolean isPastPoint(double pastPointTicks) {

//  Tests if the pos is between the input and the SetPoint.
//  This is the best way I could find.

    if (setpointTicks < inputs.positionTicks) {
      return (pastPointTicks + ExtendoConstants.kCloseEnough >= inputs.positionTicks);
    } else if (setpointTicks > inputs.positionTicks) {
      return (pastPointTicks - ExtendoConstants.kCloseEnough <= inputs.positionTicks);
    } else {
      return true;
    }
  }

  public void setPos(int positionTicks) {
    this.io.setPos(positionTicks);
    setpointTicks = positionTicks;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (zeroState == ZeroState.ZEROING){
      if (inputs.velocityTicksPer100ms <= ExtendoConstants.kVelocityThreshhold) {
        ZeroCounter++;
        if (ZeroCounter >= ExtendoConstants.kZeroCount) {
          double absolute = inputs.absoluteTicks;
          double offset = absolute - ExtendoConstants.kZeroTicks;
          io.setSelectedSensorPos(offset);
          logger.info("Abs: {}, Zero Pos: {}, Offset: {}", absolute, ExtendoConstants.kZeroTicks, offset);
          this.io.setPct(0);
          zeroState = ZeroState.IDLE;
          ZeroCounter = 0;
          io.setSupplyCurrentLimit(ExtendoConstants.getExtendoSupplyLimitConfig());
        }
      } else {
        ZeroCounter = 0;
      }
    }

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

  public enum ZeroState {
    IDLE,
    ZEROING;
  }
}
