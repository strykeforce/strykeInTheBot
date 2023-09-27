package frc.robot.subsystems.hand;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.HandConstants;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem.GamePiece;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem.TargetLevel;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
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
  private double ejectSpeed;
  private double intakeSpeed;
  private int ejectCountsNeeded;
  private DigitalInput gamePieceDI = new DigitalInput(0);

  public HandSubsystem(HandIO io) {
    this.io = io;
  }

  public HandStates getState() {
    return currState;
  }

  public void grabPiece(PieceSource pieceSource) {
    logger.info("{} -> WAITING ({})", currState, pieceSource);
    currState = HandStates.WAITING;

    switch (pieceSource) {
      case FLOOR:
        intakeSpeed = HandConstants.kWaitingFloorSpeed;
        break;
      case SUBSTATION:
        intakeSpeed = HandConstants.kWaitingSubstationSpeed;
        break;
      default:
        logger.info("Unknown piece source: {}", pieceSource);
        break;
    }
  }

  public void autoHoldCube() {
    currState = HandStates.CUBE;
  }

  public void grabFromFloor() {
    logger.info("{} -> WAITING ({})", currState);
    currState = HandStates.WAITING;

    intakeSpeed = HandConstants.kWaitingFloorSpeed;
  }

  public void grabFromSubstation() {
    logger.info("{} -> WAITING", currState);
    currState = HandStates.WAITING;

    intakeSpeed = HandConstants.kWaitingSubstationSpeed;
  }

  public void ejectPiece(GamePiece gamePiece, TargetLevel targetLevel) {

    if (gamePiece == GamePiece.CUBE) {
      switch (targetLevel) {
        case LOW:
          ejectSpeed = HandConstants.kHandEjectCubeSpeedL1;
          ejectCountsNeeded = HandConstants.kHandEjectCubeCountsL1;
          break;
        case MID:
          ejectSpeed = HandConstants.kHandEjectCubeSpeedL2;
          ejectCountsNeeded = HandConstants.kHandEjectCubeCountsL2;
          break;
        case HIGH:
          ejectSpeed = HandConstants.kHandEjectCubeSpeedL3;
          ejectCountsNeeded = HandConstants.kHandEjectCubeCountsL3;
          break;
        case NONE:
          logger.info("Target Level is NONE");
          break;
      }
    } else if (gamePiece == GamePiece.CONE) {
      switch (targetLevel) {
        case LOW:
          ejectSpeed = HandConstants.kHandEjectConeSpeedL1;
          ejectCountsNeeded = HandConstants.kHandEjectConeCountsL1;
          break;
        case MID:
          ejectSpeed = HandConstants.kHandEjectConeSpeedL2;
          ejectCountsNeeded = HandConstants.kHandEjectConeCountsL2;
          break;
        case HIGH:
          // ejectSpeed = HandConstants.kHandEjectConeSpeedL3;
          // ejectLoopCounts = HandConstants.kHandEjectConeCountsL3;
          break;
        case NONE:
          logger.info("Target Level is NONE");
          break;
      }
    }
    logger.info("{} -> EJECT", currState);
    currState = HandStates.EJECT;
  }

  // For compatability with the original RobotStateSubsystem
  public void ejectPiece(
      RobotStateSubsystem.GamePiece gamePiece, RobotStateSubsystem.TargetLevel targetLevel) {

    if (gamePiece == RobotStateSubsystem.GamePiece.CUBE) {
      switch (targetLevel) {
        case LOW:
          ejectSpeed = HandConstants.kHandEjectCubeSpeedL1;
          ejectCountsNeeded = HandConstants.kHandEjectCubeCountsL1;
          break;
        case MID:
          ejectSpeed = HandConstants.kHandEjectCubeSpeedL2;
          ejectCountsNeeded = HandConstants.kHandEjectCubeCountsL2;
          break;
        case HIGH:
          ejectSpeed = HandConstants.kHandEjectCubeSpeedL3;
          ejectCountsNeeded = HandConstants.kHandEjectCubeCountsL3;
          break;
        case NONE:
          logger.info("Target Level is NONE");
          break;
      }
    } else if (gamePiece == RobotStateSubsystem.GamePiece.CONE) {
      switch (targetLevel) {
        case LOW:
          ejectSpeed = HandConstants.kHandEjectConeSpeedL1;
          ejectCountsNeeded = HandConstants.kHandEjectConeCountsL1;
          break;
        case MID:
          ejectSpeed = HandConstants.kHandEjectConeSpeedL2;
          ejectCountsNeeded = HandConstants.kHandEjectConeCountsL2;
          break;
        case HIGH:
          // ejectSpeed = HandConstants.kHandEjectConeSpeedL3;
          // ejectLoopCounts = HandConstants.kHandEjectConeCountsL3;
          break;
        case NONE:
          logger.info("Target Level is NONE");
          break;
      }
    }
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
    if (!gamePieceDI.get()) {
      hasCubeStableCounts++;
    } else hasCubeStableCounts = 0;
    return hasCubeStableCounts > HandConstants.kHasCubeStableCounts;
  }

  public boolean hasGamePiece() {
    return hasCone() || hasCube();
  }

  public boolean isFinished() {
    return currState == HandStates.IDLE;
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
        io.setPct(intakeSpeed);
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
        if (ejectStableCounts < ejectCountsNeeded) {
          io.setPct(ejectSpeed);
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
    EJECT,
  }

  public enum PieceSource {
    FLOOR,
    SUBSTATION
  }

  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("State", () -> getState().ordinal()),
        new Measure("Digital Input", () -> gamePieceDI.get() ? 1 : 0));
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    super.registerWith(telemetryService);
    io.registerWith(telemetryService);
  }
}
