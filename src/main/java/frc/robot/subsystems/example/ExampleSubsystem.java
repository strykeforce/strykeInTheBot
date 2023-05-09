package frc.robot.subsystems.example;

import frc.robot.constants.ExampleConstants;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ExampleSubsystem extends MeasurableSubsystem {
  private final ExampleIO io;
  private final ExampleIOInputsAutoLogged inputs = new ExampleIOInputsAutoLogged();
  private ExampleStates currState = ExampleStates.IDLE;
  private ExampleStates desiredState = ExampleStates.IDLE;
  private double setpointTicks = inputs.positionTicks;
  private Logger logger = LoggerFactory.getLogger(ExampleSubsystem.class);
  private org.littletonrobotics.junction.Logger advLogger =
      org.littletonrobotics.junction.Logger.getInstance();

  public ExampleSubsystem(ExampleIO io) {
    this.io = io;
  }

  public ExampleStates getState() {
    return currState;
  }

  public void zero() {
    double absolute = inputs.absoluteTicks;
    double offset = absolute - ExampleConstants.kZeroTicks;
    io.setSelectedSensorPos(offset);
    logger.info("Abs: {}, Zero Pos: {}, Offset: {}", absolute, ExampleConstants.kZeroTicks, offset);
  }

  public void open() {
    logger.info("OPEN: {} -> TRANSITION", currState);
    desiredState = ExampleStates.OPEN;
    currState = ExampleStates.TRANSITION;
    io.setPos(ExampleConstants.kOpenPos);
    setpointTicks = ExampleConstants.kOpenPos;
  }

  public void hold() {
    logger.info("HOLD: {} -> TRANSITION", currState);
    desiredState = ExampleStates.HOLD;
    currState = ExampleStates.TRANSITION;
    io.setPos(ExampleConstants.kHoldPos);
    setpointTicks = ExampleConstants.kHoldPos;
  }

  public boolean isFinished() {
    if (currState == ExampleStates.TRANSITION) {
      switch (desiredState) {
        case IDLE:
          return true;
        case HOLD: // fall-through
        case OPEN: // fall-through
        default:
          return Math.abs(setpointTicks - inputs.positionTicks) <= ExampleConstants.kCloseEnough;
      }
    } else {
      return true;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    advLogger.processInputs("Example", inputs);

    switch (currState) {
      case IDLE:
        break;
      case HOLD:
        break;
      case OPEN:
        break;
      case TRANSITION:
        if (isFinished()) {
          logger.info("{} -> {}", currState, desiredState);
          currState = desiredState;
        }
        break;
      default:
    }

    // Log Outputs
    advLogger.recordOutput("Example/currState", currState.ordinal());
    advLogger.recordOutput("Example/setpointTicks", setpointTicks);
  }

  // States
  public enum ExampleStates {
    IDLE,
    HOLD,
    OPEN,
    TRANSITION
  }

  // Grapher Stuff
  @Override
  public Set<Measure> getMeasures() {
    return Set.of(new Measure("State", () -> getState().ordinal()));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    io.registerWith(telemetryService);
  }
}
