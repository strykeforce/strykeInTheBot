package frc.robot.subsystems.shoulder;

import frc.robot.constants.ShoulderConstants;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ShoulderSubsystem extends MeasurableSubsystem {
  private final ShoulderIO io;
  private final ShoulderIOInputsAutoLogged inputs = new ShoulderIOInputsAutoLogged();
  private ShoulderStates currState = ShoulderStates.IDLE;
  private ShoulderStates desiredState = ShoulderStates.IDLE;
  private double setpointTicks = inputs.positionTicks;
  private Logger logger = LoggerFactory.getLogger(ShoulderSubsystem.class);
  private org.littletonrobotics.junction.Logger advLogger =
      org.littletonrobotics.junction.Logger.getInstance();

  public ShoulderSubsystem(ShoulderIO io) {
    this.io = io;
  }

  public ShoulderStates getState() {
    return currState;
  }

  public void zero() {

    if (inputs.isFwdLimitSwitchClosed) {
      double absolute = inputs.absoluteTicks;
      double offset = absolute - ShoulderConstants.kZeroTicks;
      io.setSelectedSensorPos(offset * (1 / ShoulderConstants.kShoulderGearRatio));
      logger.info(
          "Abs: {}, Zero Pos: {}, Offset: {}", absolute, ShoulderConstants.kZeroTicks, offset);
    } else {
      logger.error("Limit Switch Not Aligned", getName(), advLogger);
      io.disableOutput();
    }
  }

  public void setShoulderPosition(double position) {
    io.setSelectedSensorPos(position);
    setpointTicks = position;
  }

  public boolean isFinished() {
    if (currState == ShoulderStates.TRANSITION) {
      switch (desiredState) {
        case IDLE:
          return true;
        default:
          return Math.abs(setpointTicks - inputs.positionTicks) <= ShoulderConstants.kCloseEnough;
      }
    } else {
      return true;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    advLogger.processInputs("Shoulder", inputs);

    switch (currState) {
      case IDLE:
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
    advLogger.recordOutput("Shoulder/currState", currState.ordinal());
    advLogger.recordOutput("Shoulder/setpointTicks", setpointTicks);
  }

  // States
  public enum ShoulderStates {
    IDLE,
    TRANSITION
  }

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
