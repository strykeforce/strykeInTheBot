package frc.robot.subsystems.Arm;

import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.Extendo.ExtendoSubsystem;
import frc.robot.subsystems.Wrist.WristSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.GamePiece;
import frc.robot.subsystems.shoulder.ShoulderSubsystem;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ArmSubsystem extends MeasurableSubsystem {

  private final ShoulderSubsystem shoulderSubsystem;
  private final ExtendoSubsystem extendoSubsystem;
  private final WristSubsystem wristSubsystem;
  private Logger logger = LoggerFactory.getLogger(ArmSubsystem.class);
  private ArmState currState;
  private ArmState desiredState;
  private ArmState finalState;
  private boolean useFinal;

  public ArmSubsystem(
      ShoulderSubsystem shoulderSubsystem,
      ExtendoSubsystem extendoSubsystem,
      WristSubsystem wristSubsystem) {
    this.shoulderSubsystem = shoulderSubsystem;
    this.extendoSubsystem = extendoSubsystem;
    this.wristSubsystem = wristSubsystem;
    currState = ArmState.STOW;
    desiredState = ArmState.STOW;
    finalState = ArmState.STOW;
    useFinal = false;
  }

  public ArmState getcurrState() {
    return currState;
  }

  public ArmState getdesiredState() {
    return desiredState;
  }

  public ArmState getfinalState() {
    return finalState;
  }

  public void zero() {
    shoulderSubsystem.zero();
    extendoSubsystem.zero();
    wristSubsystem.zero();
  }

  public void stow() {
    useFinal = false;
    if (currState != ArmState.STOW) {
      shoulderSubsystem.setPos(ArmConstants.kStowShoulderPos);
      extendoSubsystem.setPos(ArmConstants.kStowExtendoPos);
      wristSubsystem.setPos(ArmConstants.kStowWristPos);
      currState = ArmState.PARALLEL_TRANS;
      desiredState = ArmState.STOW;
    }
  }

  public void high(GamePiece piece) {
    useFinal = false;
    if (piece != GamePiece.NONE) {
      if (currState == ArmState.STOW) {
        shoulderSubsystem.setPos(ArmConstants.kPlaceShoulderPos);
        currState = ArmState.STOW_TO_HIGH;
        if (piece == GamePiece.CUBE) {
          desiredState = ArmState.HIGH_CUBE;
          wristSubsystem.setPos(ArmConstants.kCubePlaceWristPos);
        } else {
          desiredState = ArmState.HIGH_CONE;
          wristSubsystem.setPos(ArmConstants.kConePlaceWristPos);
        }
      } else if (currState == ArmState.MID_CUBE || currState == ArmState.MID_CONE) {
        currState = ArmState.PARALLEL_TRANS;
        extendoSubsystem.setPos(ArmConstants.kHighExtendoPos);
        if (piece == GamePiece.CONE) {
          desiredState = ArmState.HIGH_CONE;
          wristSubsystem.setPos(ArmConstants.kConePlaceWristPos);
        } else {
          desiredState = ArmState.HIGH_CUBE;
          wristSubsystem.setPos(ArmConstants.kConePlaceWristPos);
        }
      } else {
        this.stow();
        if (piece == GamePiece.CUBE) {
          finalState = ArmState.HIGH_CUBE;
        } else {
          finalState = ArmState.HIGH_CONE;
        }
        useFinal = true;
      }
    } else {
      logger.info("cannot place nonexistent game piece");
    }
  }

  public void mid(GamePiece piece) {
    useFinal = false;
    if (piece != GamePiece.NONE) {
      if (currState == ArmState.STOW) {
        shoulderSubsystem.setPos(ArmConstants.kPlaceShoulderPos);
        currState = ArmState.STOW_TO_MID;
        if (piece == GamePiece.CUBE) {
          desiredState = ArmState.MID_CUBE;
          wristSubsystem.setPos(ArmConstants.kCubePlaceWristPos);
        } else {
          desiredState = ArmState.LOW_CONE;
          wristSubsystem.setPos(ArmConstants.kConePlaceWristPos);
        }
      } else if (currState == ArmState.HIGH_CUBE || currState == ArmState.HIGH_CONE) {
        currState = ArmState.PARALLEL_TRANS;
        extendoSubsystem.setPos(ArmConstants.kHighExtendoPos);
        if (piece == GamePiece.CONE) {
          desiredState = ArmState.MID_CONE;
          wristSubsystem.setPos(ArmConstants.kConePlaceWristPos);
        } else {
          desiredState = ArmState.MID_CUBE;
          wristSubsystem.setPos(ArmConstants.kCubePlaceWristPos);
        }
      } else {
        this.stow();
        if (piece == GamePiece.CUBE) {
          finalState = ArmState.MID_CUBE;
        } else {
          finalState = ArmState.MID_CONE;
        }
        useFinal = true;
      }
    } else {
      logger.info("cannot place nonexistent game piece");
    }
  }

  public void low(GamePiece piece) {
    useFinal = false;
    if (piece != GamePiece.NONE) {
      if (currState == ArmState.STOW) {
        shoulderSubsystem.setPos(ArmConstants.kLowPlaceShoulderPos);
        extendoSubsystem.setPos(ArmConstants.kLowExtendoPos);
        currState = ArmState.PARALLEL_TRANS;
        if (piece == GamePiece.CUBE) {
          desiredState = ArmState.LOW_CUBE;
          wristSubsystem.setPos(ArmConstants.kCubeLowPlaceWristPos);
        } else {
          desiredState = ArmState.LOW_CONE;
          wristSubsystem.setPos(ArmConstants.kConeLowPlaceWristPos);
        }
      } else {
        this.stow();
        if (piece == GamePiece.CUBE) {
          finalState = ArmState.LOW_CUBE;
        } else {
          finalState = ArmState.LOW_CONE;
        }
        useFinal = true;
      }
    } else {
      logger.info("cannot place nonexistent game piece");
    }
  }

  public void shelf(GamePiece piece) {
    useFinal = false;
    if (piece != GamePiece.NONE) {
      if (currState == ArmState.STOW) {
        currState = ArmState.PARALLEL_TRANS;
        if (piece == GamePiece.CUBE) {
          desiredState = ArmState.SHELF_CUBE;
          shoulderSubsystem.setPos(ArmConstants.kCubeShelfShoulderPos);
          extendoSubsystem.setPos(ArmConstants.kCubeShelfExtendoPos);
          wristSubsystem.setPos(ArmConstants.kCubeShelfWristPos);
        } else {
          desiredState = ArmState.SHELF_CONE;
          shoulderSubsystem.setPos(ArmConstants.kConeShelfShoulderPos);
          extendoSubsystem.setPos(ArmConstants.kConeShelfExtendoPos);
          wristSubsystem.setPos(ArmConstants.kConeShelfWristPos);
        }
      } else {
        this.stow();
        if (piece == GamePiece.CUBE) {
          finalState = ArmState.SHELF_CUBE;
        } else {
          finalState = ArmState.SHELF_CONE;
        }
        useFinal = true;
      }
    } else {
      logger.info("cannot pickup nonexistent game piece");
    }
  }

  public void floor(GamePiece piece, boolean isUpright) {
    useFinal = false;
    // if it's a cube, set isUpright to whatever! freedom!!!
    if (piece != GamePiece.NONE) {
      if (currState == ArmState.STOW) {
        currState = ArmState.PARALLEL_TRANS;
        if (piece == GamePiece.CUBE) {
          desiredState = ArmState.FLOOR_CUBE;
          shoulderSubsystem.setPos(ArmConstants.kCubeFloorShoulderPos);
          extendoSubsystem.setPos(ArmConstants.kCubeFloorExtendoPos);
          wristSubsystem.setPos(ArmConstants.kCubeFloorWristPos);
        } else {
          if (isUpright) {
            desiredState = ArmState.FLOOR_CONE_UPRIGHT;
            shoulderSubsystem.setPos(ArmConstants.kConeFloorUprightShoulderPos);
            extendoSubsystem.setPos(ArmConstants.kConeFloorUprightExtendoPos);
            wristSubsystem.setPos(ArmConstants.kConeFloorUprightWristPos);
          } else {
            desiredState = ArmState.FLOOR_CONE;
            shoulderSubsystem.setPos(ArmConstants.kConeFloorShoulderPos);
            extendoSubsystem.setPos(ArmConstants.kConeFloorExtendoPos);
            wristSubsystem.setPos(ArmConstants.kConeFloorWristPos);
          }
        }
      } else {
        this.stow();
        if (piece == GamePiece.CUBE) {
          finalState = ArmState.FLOOR_CUBE;
        } else {
          if (isUpright) {
            finalState = ArmState.FLOOR_CONE_UPRIGHT;
          } else {
            finalState = ArmState.FLOOR_CONE;
          }
        }
        useFinal = true;
      }
    } else {
      logger.info("cannot pickup nonexistent game piece");
    }
  }

  public void yoshi(GamePiece piece, boolean isUpright) {
    useFinal = false;
    // if it's a cube, set isUpright to whatever! freedom!!!
    if (piece != GamePiece.NONE) {
      if (currState == ArmState.FLOOR_CUBE
          || currState == ArmState.FLOOR_CONE
          || currState == ArmState.FLOOR_CONE_UPRIGHT) {
        currState = ArmState.PARALLEL_TRANS;
        if (piece == GamePiece.CUBE) {
          desiredState = ArmState.YOSHIED;
          shoulderSubsystem.setPos(ArmConstants.kCubeFloorShoulderPos);
          extendoSubsystem.setPos(ArmConstants.kYoshiPos);
          wristSubsystem.setPos(ArmConstants.kCubeFloorWristPos);
        } else {
          if (isUpright) {
            desiredState = ArmState.YOSHIED;
            shoulderSubsystem.setPos(ArmConstants.kConeFloorUprightShoulderPos);
            extendoSubsystem.setPos(ArmConstants.kYoshiPos);
            wristSubsystem.setPos(ArmConstants.kConeFloorUprightWristPos);
          } else {
            desiredState = ArmState.YOSHIED;
            shoulderSubsystem.setPos(ArmConstants.kConeFloorShoulderPos);
            extendoSubsystem.setPos(ArmConstants.kYoshiPos);
            wristSubsystem.setPos(ArmConstants.kConeFloorWristPos);
          }
        }
      }
    } else {
      logger.info("cannot pickup nonexistent game piece");
    }
  }

  private boolean isFinishedP() {
    if (shoulderSubsystem.isFinished()
        && extendoSubsystem.isFinished()
        && wristSubsystem.isFinished()) {
      return true;
    }
    return false;
  }

  public boolean isFinished() {
    return currState == desiredState && !useFinal;
  }

  @Override
  public void periodic() {
    if (currState == ArmState.STOW_TO_HIGH
        && shoulderSubsystem.isPastPoint(ArmConstants.highShoulderpastPointTicks)) {
      extendoSubsystem.setPos(ArmConstants.kHighExtendoPos);
    }

    if (currState == ArmState.STOW_TO_MID
        && shoulderSubsystem.isPastPoint(ArmConstants.midShoulderpastPointTicks)) {
      extendoSubsystem.setPos(ArmConstants.kMidExtendoPos);
    }

    if (this.isFinishedP()) {
      currState = desiredState;
      if (useFinal && currState == ArmState.STOW) {
        switch (finalState) {
          case HIGH_CUBE:
            useFinal = false;
            high(GamePiece.CUBE);
            break;
          case HIGH_CONE:
            useFinal = false;
            high(GamePiece.CONE);
            break;
          case MID_CUBE:
            useFinal = false;
            mid(GamePiece.CUBE);
            break;
          case MID_CONE:
            useFinal = false;
            mid(GamePiece.CONE);
            break;
          case LOW_CUBE:
            useFinal = false;
            low(GamePiece.CUBE);
            break;
          case LOW_CONE:
            useFinal = false;
            low(GamePiece.CONE);
            break;
          case SHELF_CUBE:
            useFinal = false;
            shelf(GamePiece.CUBE);
            break;
          case SHELF_CONE:
            useFinal = false;
            shelf(GamePiece.CONE);
            break;
          case FLOOR_CUBE:
            useFinal = false;
            floor(GamePiece.CUBE, true);
            break;
          case FLOOR_CONE:
            useFinal = false;
            floor(GamePiece.CONE, false);
            break;
          case FLOOR_CONE_UPRIGHT:
            useFinal = false;
            floor(GamePiece.CONE, true);
            break;
          case YOSHIED:
            // fall through
          case PARALLEL_TRANS:
            // fall through
          case STOW_TO_HIGH:
            // fall through
          case STOW:
            // fall through
          default:
            useFinal = false;
            break;
        }
      }
    }
  }

  public enum ArmState {
    STOW,
    HIGH_CUBE,
    HIGH_CONE,
    MID_CUBE,
    MID_CONE,
    LOW_CUBE,
    LOW_CONE,
    SHELF_CUBE,
    SHELF_CONE,
    FLOOR_CUBE,
    FLOOR_CONE,
    FLOOR_CONE_UPRIGHT,
    YOSHIED,
    PARALLEL_TRANS,
    STOW_TO_HIGH,
    STOW_TO_MID
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("currState", () -> currState.ordinal()),
        new Measure("desiredState", () -> desiredState.ordinal()),
        new Measure("finalState", () -> finalState.ordinal()),
        new Measure("State", () -> useFinal ? 1 : 0));
  }
}
