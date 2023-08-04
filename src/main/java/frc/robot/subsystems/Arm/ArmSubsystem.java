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
    shoulderSubsystem.setPos(ArmConstants.kStowShoulderPos);
    extendoSubsystem.setPos(ArmConstants.kStowExtendoPos);
    wristSubsystem.setPos(ArmConstants.kStowWristPos);
    currState = ArmState.PARALLEL_TRANS;
    desiredState = ArmState.STOW;
  }

  public void high(gamePiece piece) {
    if (currState == ArmState.STOW) {
      shoulderSubsystem.setPos(ArmConstants.kPlaceShoulderPos);
      currState = ArmState.STOW_TO_HIGH;
      if (piece == gamePiece.CUBE) {
        desiredState = ArmState.HIGH_CUBE;
        wristSubsystem.setPos(ArmConstants.kCubePlaceWristPos);
      } else {
        desiredState = ArmState.HIGH_CONE;
        wristSubsystem.setPos(ArmConstants.kConePlaceWristPos);
      }
    } else {
      this.stow();
      desiredState = ArmState.STOW;
      finalState = ArmState.HIGH_CUBE;
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
            
        }
      }
    }

    if (currState == ArmState.STOW_TO_HIGH && shoulderSubsystem.isPastPoint(ArmConstants.highShoulderpastPointTicks)) {
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
    SHELF_CONE,
    SHELF_CUBE,
    FLOOR_CONE,
    FLOOR_CONE_UPRIGHT,
    FLOOR_CUBE,
    PARALLEL_TRANS,
    STOW_TO_HIGH
  }

  @Override
  public Set<Measure> getMeasures() {
    // TODO Fill in measures for grapher
    return null;
  }
}
