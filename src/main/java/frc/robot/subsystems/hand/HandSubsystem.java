package frc.robot.subsystems.hand;

import frc.robot.constants.HandConstants;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class HandSubsystem extends MeasurableSubsystem {
  private final HandIO io;
  private final HandIOInputsAutoLogged inputs = new HandIOInputsAutoLogged();
  private HandStates currState = HandStates.IDLE;
  private Logger logger = LoggerFactory.getLogger(HandSubsystem.class);
  private org.littletonrobotics.junction.Logger advLogger =
      org.littletonrobotics.junction.Logger.getInstance();
  private int hasConeStableCounts;
  private int hasCubeStableCounts;
  private int ejectStableCounts = 0;

  public HandSubsystem(HandIO io) {
    this.io = io;
  }

  public HandStates getState() {
    return currState;
  }

  public void grabPiece() {
    logger.info("{} -> WAITING", currState);
    currState = HandStates.WAITING;
  }

  public void ejectPiece() {
    logger.info("{} -> EJECT", currState);
    currState = HandStates.EJECT;
  }

  public void idle() {
    if (currState == HandStates.WAITING) {
      logger.info("WAITING -> IDLE");
      currState = HandStates.IDLE;
    } else logger.info("failed to IDLE: hand has a piece");
  }

  public boolean hasCone() {
    if (inputs.isFwdLimitSwitchClosed
        && Math.abs(inputs.velocityTicksPer100ms) <= HandConstants.kConeVelLimit) {
      hasConeStableCounts++;
    } else hasConeStableCounts = 0;
    return hasConeStableCounts > HandConstants.kHasConeStableCounts;
  }

  public boolean hasCube() {
    if (inputs.isRevLimitSwitchClosed) {
      hasCubeStableCounts++;
    } else hasCubeStableCounts = 0;
    return hasCubeStableCounts > HandConstants.kHasCubeStableCounts;
  }

  public boolean hasGamePiece() {
    return hasCone() || hasCube();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    advLogger.processInputs("Hand", inputs);

    switch (currState) {
      case IDLE:
        io.setPct(0.0);
        break;
      case WAITING:
        io.setPct(HandConstants.kWaitingSpeed);
        if (hasCone()) {
          logger.info("WAITING -> CONE");
          currState = HandStates.CONE;
        }
        if (hasCube()) {
          logger.info("WAITING -> CUBE");
          currState = HandStates.CUBE;
        }
        break;
      case CONE:
        io.setPct(HandConstants.kConeSpeed);
        break;
      case CUBE:
        io.setPct(HandConstants.kCubeSpeed);
        break;
      case EJECT:
        if (ejectStableCounts < HandConstants.kEjectStableCounts) {
          io.setPct(-HandConstants.kWaitingSpeed);
          ejectStableCounts++;
        } else {
          ejectStableCounts = 0;
          logger.info("EJECT -> IDLE");
          currState = HandStates.IDLE;
        }
        break;
    }
  }

  public enum HandStates {
    IDLE,
    WAITING,
    CONE,
    CUBE,
    EJECT
  }

  public Set<Measure> getMeasures() {
    return Set.of(new Measure("State", () -> getState().ordinal()));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    io.registerWith(telemetryService);
  }
}
