package frc.robot.subsystems.Arm;

import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.Extendo.ExtendoSubsystem;
import frc.robot.subsystems.RobotState.RobotStateSubsystem.gamePiece;
import frc.robot.subsystems.Wrist.WristSubsystem;
import frc.robot.subsystems.shoulder.ShoulderSubsystem;
import java.util.Set;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ArmSubsystem extends MeasurableSubsystem {

  private final ShoulderSubsystem shoulderSubsystem;
  private final ExtendoSubsystem extendoSubsystem;
  private final WristSubsystem wristSubsystem;
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

  public void zero() {
    shoulderSubsystem.zero();
    extendoSubsystem.zero();
    wristSubsystem.zero();
  }

  public void stow() {
    if (currState != ArmState.STOW) {
      shoulderSubsystem.setPos(ArmConstants.kStowShoulderPos);
      extendoSubsystem.setPos(ArmConstants.kStowExtendoPos);
      wristSubsystem.setPos(ArmConstants.kStowWristPos);
      currState = ArmState.PARALLEL_TRANS;
      desiredState = ArmState.STOW;
    }
  }

  public void high(gamePiece piece) {
    if (currState == ArmState.STOW) {
      shoulderSubsystem.setPos(ArmConstants.kPlaceShoulderPos);
      extendoSubsystem.setPos(ArmConstants.kHighExtendoPos);
      currState = ArmState.STOW_TO_HIGH;
      if (piece == gamePiece.CUBE) {
        desiredState = ArmState.HIGH_CUBE;
        wristSubsystem.setPos(ArmConstants.kCubePlaceWristPos);
      } else {
        desiredState = ArmState.HIGH_CONE;
        wristSubsystem.setPos(ArmConstants.kConePlaceWristPos);
      }
    } else if (currState == ArmState.MID_CUBE) {
      currState = ArmState.PARALLEL_TRANS;
      extendoSubsystem.setPos(ArmConstants.kHighExtendoPos);
      if (piece == gamePiece.CONE) {
        desiredState = ArmState.HIGH_CONE;
        wristSubsystem.setPos(ArmConstants.kConePlaceWristPos);
      } else {
        desiredState = ArmState.HIGH_CUBE;
      }
    } else if (currState == ArmState.MID_CONE) {
      currState = ArmState.PARALLEL_TRANS;
      extendoSubsystem.setPos(ArmConstants.kHighExtendoPos);
      if (piece == gamePiece.CUBE) {
        desiredState = ArmState.HIGH_CUBE;
        wristSubsystem.setPos(ArmConstants.kConePlaceWristPos);
      } else {
        desiredState = ArmState.HIGH_CONE;
      }
    } else {
      this.stow();
      if (piece == gamePiece.CUBE) {
        finalState = ArmState.HIGH_CUBE;
      } else {
        finalState = ArmState.HIGH_CONE;
      }
      useFinal = true;
    }
  }

  public void mid(gamePiece piece) {
    if (currState == ArmState.STOW) {
      shoulderSubsystem.setPos(ArmConstants.kPlaceShoulderPos);
      extendoSubsystem.setPos(ArmConstants.kMidExtendoPos);
      currState = ArmState.PARALLEL_TRANS;
      if (piece == gamePiece.CUBE) {
        desiredState = ArmState.MID_CUBE;
        wristSubsystem.setPos(ArmConstants.kCubePlaceWristPos);
      } else {
        desiredState = ArmState.LOW_CONE;
        wristSubsystem.setPos(ArmConstants.kConePlaceWristPos);
      }
    } else if (currState == ArmState.HIGH_CUBE) {
      currState = ArmState.PARALLEL_TRANS;
      extendoSubsystem.setPos(ArmConstants.kHighExtendoPos);
      if (piece == gamePiece.CUBE) {
        desiredState = ArmState.MID_CONE;
        wristSubsystem.setPos(ArmConstants.kConePlaceWristPos);
      } else {
        desiredState = ArmState.MID_CUBE;
      }
    } else if (currState == ArmState.HIGH_CONE) {
      currState = ArmState.PARALLEL_TRANS;
      extendoSubsystem.setPos(ArmConstants.kHighExtendoPos);
      if (piece == gamePiece.CUBE) {
        desiredState = ArmState.MID_CUBE;
        wristSubsystem.setPos(ArmConstants.kConePlaceWristPos);
      } else {
        desiredState = ArmState.MID_CONE;
      }
    } else {
      this.stow();
      if (piece == gamePiece.CUBE) {
        finalState = ArmState.MID_CUBE;
      } else {
        finalState = ArmState.MID_CONE;
      }
      useFinal = true;
    }
  }

  public void low(gamePiece piece) {
    if (currState == ArmState.STOW) {
      shoulderSubsystem.setPos(ArmConstants.kLowPlaceShoulderPos);
      extendoSubsystem.setPos(ArmConstants.kMidExtendoPos);
      currState = ArmState.PARALLEL_TRANS;
      if (piece == gamePiece.CUBE) {
        desiredState = ArmState.LOW_CUBE;
        wristSubsystem.setPos(ArmConstants.kCubeLowPlaceWristPos);
      } else {
        desiredState = ArmState.LOW_CONE;
        wristSubsystem.setPos(ArmConstants.kConeLowPlaceWristPos);
      }
    } else {
      this.stow();
      if (piece == gamePiece.CUBE) {
        finalState = ArmState.LOW_CUBE;
      } else {
        finalState = ArmState.LOW_CONE;
      }
      useFinal = true;
    }
  }

  public void shelf(gamePiece piece) {
    if (currState == ArmState.STOW) {
      currState = ArmState.PARALLEL_TRANS;
      if (piece == gamePiece.CUBE) {
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
      if (piece == gamePiece.CUBE) {
        finalState = ArmState.SHELF_CUBE;
      } else {
        finalState = ArmState.SHELF_CONE;
      }
      useFinal = true;
    }
  }

  public void floor(gamePiece piece, boolean isUpright) {
    // if it's a cube, set isUpright to whatever! freedom!!!
    if (currState == ArmState.STOW) {
      currState = ArmState.PARALLEL_TRANS;
      if (piece == gamePiece.CUBE) {
        desiredState = ArmState.FLOOR_CUBE;
        shoulderSubsystem.setPos(ArmConstants.kCubeFloorShoulderPos);
        extendoSubsystem.setPos(ArmConstants.kCubeFloorExtendoPos);
        wristSubsystem.setPos(ArmConstants.kCubeFloorWristPos);
      } else {
        desiredState = ArmState.FLOOR_CONE;
        if (isUpright) {
          shoulderSubsystem.setPos(ArmConstants.kConeFloorUprightShoulderPos);
          extendoSubsystem.setPos(ArmConstants.kConeFloorUprightExtendoPos);
          wristSubsystem.setPos(ArmConstants.kConeFloorUprightWristPos);
        } else {
          shoulderSubsystem.setPos(ArmConstants.kConeFloorShoulderPos);
          extendoSubsystem.setPos(ArmConstants.kConeFloorExtendoPos);
          wristSubsystem.setPos(ArmConstants.kConeFloorWristPos);
        }
      }
    } else {
      this.stow();
      if (piece == gamePiece.CUBE) {
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
  }

  public void yoshi() {
    if (currState == ArmState.FLOOR_CUBE) {
      currState = ArmState.PARALLEL_TRANS;
      desiredState = ArmState.YOSHIED;
      extendoSubsystem.setPos(ArmConstants.kYoshiPos);
      finalState = ArmState.FLOOR_CUBE;
    }
    if (currState == ArmState.FLOOR_CONE) {
      currState = ArmState.PARALLEL_TRANS;
      desiredState = ArmState.YOSHIED;
      extendoSubsystem.setPos(ArmConstants.kYoshiPos);
      finalState = ArmState.FLOOR_CONE;
    }
    if (currState == ArmState.FLOOR_CONE_UPRIGHT) {
      currState = ArmState.PARALLEL_TRANS;
      desiredState = ArmState.YOSHIED;
      extendoSubsystem.setPos(ArmConstants.kYoshiPos);
      finalState = ArmState.FLOOR_CONE_UPRIGHT;
    }
  }

  public boolean isFinished() {
    if (shoulderSubsystem.isFinished()
        && extendoSubsystem.isFinished()
        && wristSubsystem.isFinished()) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    if (this.isFinished()) {
      currState = desiredState;
      if (useFinal) {
        switch (finalState) {
          case HIGH_CUBE:
            useFinal = false;
            high(gamePiece.CUBE);
            break;
          case HIGH_CONE:
            useFinal = false;
            high(gamePiece.CONE);
            break;
          case MID_CUBE:
            useFinal = false;
            mid(gamePiece.CUBE);
            break;
          case MID_CONE:
            useFinal = false;
            mid(gamePiece.CONE);
            break;
          case LOW_CUBE:
            useFinal = false;
            low(gamePiece.CUBE);
            break;
          case LOW_CONE:
            useFinal = false;
            low(gamePiece.CONE);
            break;
          case SHELF_CUBE:
            useFinal = false;
            shelf(gamePiece.CUBE);
            break;
          case SHELF_CONE:
            useFinal = false;
            shelf(gamePiece.CONE);
            break;
          case FLOOR_CUBE:
            useFinal = false;
            floor(gamePiece.CUBE, true);
            break;
          case FLOOR_CONE:
            useFinal = false;
            floor(gamePiece.CONE, false);
            break;
          case FLOOR_CONE_UPRIGHT:
            useFinal = false;
            floor(gamePiece.CONE, true);
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

    if (currState == ArmState.STOW_TO_HIGH
        && shoulderSubsystem.isPastPoint(ArmConstants.highShoulderpastPointTicks)) {
      extendoSubsystem.setPos(ArmConstants.kHighExtendoPos);
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
    STOW_TO_HIGH
  }

  @Override
  public Set<Measure> getMeasures() {
    // TODO Fill in measures for grapher
    return null;
  }
}
