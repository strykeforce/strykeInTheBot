package frc.robot.subsystems.robotState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.hand.HandSubsystem;
import frc.robot.subsystems.hand.HandSubsystem.HandStates;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class RobotStateSubsystem extends SubsystemBase {
  private DriveSubsystem driveSubsystem;
  private ArmSubsystem armSubsystem;
  private HandSubsystem handSubsystem;
  private Alliance allianceColor = DriverStation.getAlliance();
  private Logger logger = LoggerFactory.getLogger(RobotStateSubsystem.class);
  private RobotState currRobotState = RobotState.STOW;
  private RobotState nextRobotState = RobotState.STOW;
  private GamePiece currentPiece = GamePiece.NONE;
  private GamePiece targetPiece = GamePiece.NONE;
  private TargetLevel targetLevel = TargetLevel.NONE;
  private TargetCol targetCol = TargetCol.NONE;
  private double currShelfPoseX;
  private boolean isConePickupUpright = true;

  public RobotStateSubsystem(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.armSubsystem = armSubsystem;
  }

  public Alliance getAllianceColor() {
    return allianceColor;
  }

  public void setAllianceColor(Alliance alliance) {
    // logger.info("Changing to {}", alliance);
    this.allianceColor = alliance;
  }

  public RobotState getCurrRobotState() {
    return currRobotState;
  }

  private void setRobotStateLogged(RobotState robotState) {
    if (this.currRobotState != robotState) {
      logger.info("{} -> {}", this.currRobotState, robotState);
      this.currRobotState = robotState;
    }
  }

  public GamePiece getCurrentPiece() {
    return currentPiece;
  }

  public TargetLevel getTargetLevel() {
    return targetLevel;
  }

  public void setTargetLevel(TargetLevel targetLevel) {
    this.targetLevel = targetLevel;
  }

  public TargetCol getTargetCol() {
    return targetCol;
  }

  public void setTargetCol(TargetCol targetCol) {
    this.targetCol = targetCol;
  }

  public GamePiece getTargetPiece() {
    return targetPiece;
  }

  public void setTargetPiece(GamePiece targetPiece) {
    this.targetPiece = targetPiece;
  }

  public boolean isStowed() {
    return currRobotState == RobotState.STOW;
  }

  public boolean isShelfPieceCube() {
    // FIXME maybe flip isBlueAlliance boolean
    double angle = driveSubsystem.getGyroRotation2d().getRadians();
    if (isBlueAlliance()) {
      return (angle > Math.PI * -0.5) && (angle < Math.PI * 0.5);
    } else {
      return (angle < Math.PI * -0.5) || (angle > Math.PI * 0.5);
    }
  }

  public boolean isBlueAlliance() {
    return allianceColor == Alliance.Blue;
  }

  public void toStow(RobotState nextState) {
    nextRobotState = nextState;
    handSubsystem.idle();
    armSubsystem.stow();
    setRobotStateLogged(RobotState.TO_STOW);
  }

  public void toStow() {
    toStow(RobotState.STOW);
  }

  public void toFloorPickup(GamePiece targetPiece) {
    logger.info("starting floor pickup");

    this.targetPiece = targetPiece;
    if (isStowed()) {
      switch (targetPiece) {
        case CUBE:
          armSubsystem.floor(GamePiece.CUBE, true);
          handSubsystem.grabPiece();
          break;
        case CONE:
          if (isConePickupUpright) {
            armSubsystem.floor(GamePiece.CONE, true);
          } else {
            armSubsystem.floor(GamePiece.CONE, false);
          }
          handSubsystem.grabPiece();
          break;
        default:
          logger.warn("no target piece given for floor pickup!");
          break;
      }
      setRobotStateLogged(RobotState.TO_FLOOR_PICKUP);
    } else {
      toStow(RobotState.FLOOR_PICKUP);
    }
  }

  public void toFloorPickup() {
    toFloorPickup(targetPiece);
  }

  public void toAutoShelf() {
    logger.info("starting auto shelf");

    if (isStowed()) {
      if (isShelfPieceCube()) {
        armSubsystem.shelf(GamePiece.CUBE);
      } else {
        armSubsystem.shelf(GamePiece.CONE);
      }
      setRobotStateLogged(RobotState.TO_AUTO_SHELF);
    } else {
      toStow(RobotState.AUTO_SHELF);
    }
  }

  public void toManualShelf() {
    logger.info("starting manual shelf");

    if (isStowed()) {
      if (isShelfPieceCube()) {
        armSubsystem.shelf(GamePiece.CUBE);
      } else {
        armSubsystem.shelf(GamePiece.CONE);
      }
      setRobotStateLogged(RobotState.TO_MANUAL_SHELF);
    } else {
      toStow(RobotState.MANUAL_SHELF);
    }
  }

  public void toAutoScore() {
    logger.info("starting auto score");

    if (isStowed()) {
      if (currentPiece != GamePiece.NONE) {
        switch (targetLevel) {
          case LOW:
            armSubsystem.low(currentPiece);
            break;
          case MID:
            armSubsystem.mid(currentPiece);
            break;
          case HIGH:
            armSubsystem.high(currentPiece);
            break;
          default:
            break;
        }
      }
      setRobotStateLogged(RobotState.TO_AUTO_SCORE);
    } else {
      toStow(RobotState.AUTO_SCORE);
    }
  }

  public void toManualScore() {
    logger.info("starting manual score");

    if (isStowed()) {
      setRobotStateLogged(RobotState.TO_MANUAL_SCORE);
    } else {
      toStow(RobotState.MANUAL_SCORE);
    }
  }

  public void toReleaseGamepiece() {
    logger.info("starting release gamepiece");

    setRobotStateLogged(RobotState.RELEASE_GAMEPIECE);
  }

  public void clearGamePiece() {
    this.currentPiece = GamePiece.NONE;
    logger.info("Cleared Gamepiece");
  }

  public void toAutobalance(boolean isOnAllianceSide) {
    logger.info("starting autobalance");
    // FIXME: HANDLE ON ALLIANCE SIDE
    if (isStowed()) {
      // TODO: add autobalance functionality
      setRobotStateLogged(RobotState.TO_AUTOBALANCE);
    } else {
      toStow(RobotState.AUTOBALANCE);
    }
  }

  public void toPulseAutoBalance(boolean isOnAllianceSide) {
    // FIXME: IMPLEMENT THIS METHOD
  }

  @Override
  public void periodic() {
    switch (currRobotState) {
      case STOW:
        switch (nextRobotState) {
          case FLOOR_PICKUP:
            toFloorPickup();
            break;
          case AUTO_SHELF:
            toAutoShelf();
            break;
          case MANUAL_SHELF:
            toManualShelf();
            break;
          case AUTO_SCORE:
            toAutoScore();
            break;
          case MANUAL_SCORE:
            toManualScore();
            break;
          case AUTOBALANCE:
            // FIXME: logic is broken...
            toAutobalance(false);
            break;
          default:
            break;
        }
        break;
      case FLOOR_PICKUP:
        if (!handSubsystem.hasGamePiece()) break;
        currentPiece = targetPiece;
        toStow();
        break;
      case AUTO_SHELF:
        // wait to drive to correct spot
        if (!handSubsystem.hasGamePiece()) break;
        currentPiece = targetPiece;
        currShelfPoseX = driveSubsystem.getPoseMeters().getX();
        setRobotStateLogged(RobotState.SHELF_WAIT);
        break;
      case MANUAL_SHELF:
        if (!handSubsystem.hasGamePiece()) break;
        currentPiece = targetPiece;
        currShelfPoseX = driveSubsystem.getPoseMeters().getX();
        setRobotStateLogged(RobotState.SHELF_WAIT);
        break;
      case SHELF_WAIT:
        double desiredPoseX;
        if (isBlueAlliance()) {
          desiredPoseX = currShelfPoseX - ArmConstants.kShelfMove;
          if (driveSubsystem.getPoseMeters().getX() <= desiredPoseX) {
            toStow();
          }
        } else {
          desiredPoseX = currShelfPoseX + ArmConstants.kShelfMove;
          if (driveSubsystem.getPoseMeters().getX() >= desiredPoseX) {
            toStow();
          }
        }
        break;
      case AUTO_SCORE:
        break;
      case MANUAL_SCORE:
        break;
      case RELEASE_GAMEPIECE:
        if (handSubsystem.getState() == HandStates.IDLE) {
          currentPiece = GamePiece.NONE;
          toStow();
        } else handSubsystem.ejectPiece();
        break;
      case AUTOBALANCE:
        // TODO: add autobalance functionality
        break;
      case TO_STOW:
        if (!armSubsystem.isFinished()) break;
        setRobotStateLogged(RobotState.STOW);
        break;
      case TO_FLOOR_PICKUP:
        if (!armSubsystem.isFinished()) break;
        handSubsystem.grabPiece();
        setRobotStateLogged(RobotState.FLOOR_PICKUP);
        break;
      case TO_AUTO_SHELF:
        if (!armSubsystem.isFinished()) break;
        handSubsystem.grabPiece();
        // driveSubsystem.driveToPose();
        setRobotStateLogged(RobotState.AUTO_SHELF);
        break;
      case TO_MANUAL_SHELF:
        if (!armSubsystem.isFinished()) break;
        handSubsystem.grabPiece();
        setRobotStateLogged(RobotState.MANUAL_SHELF);
        break;
      case TO_AUTO_SCORE:
        if (!armSubsystem.isFinished()) break;
        // drive to pos
        setRobotStateLogged(RobotState.AUTO_SCORE);
        break;
      case TO_MANUAL_SCORE:
        if (!armSubsystem.isFinished()) break;
        setRobotStateLogged(RobotState.MANUAL_SCORE);
        break;
      case TO_AUTOBALANCE:
        // TODO: add autobalance functionality
        break;
      default:
        logger.warn("{} is an invalid robot state!", currRobotState);
        break;
    }
  }

  public enum RobotState {
    STOW,
    FLOOR_PICKUP,
    AUTO_SHELF,
    MANUAL_SHELF,
    SHELF_WAIT,
    AUTO_SCORE,
    MANUAL_SCORE,
    RELEASE_GAMEPIECE,
    AUTOBALANCE,
    TO_STOW,
    TO_FLOOR_PICKUP,
    TO_AUTO_SHELF,
    TO_MANUAL_SHELF,
    TO_AUTO_SCORE,
    TO_MANUAL_SCORE,
    TO_AUTOBALANCE,
  }

  public enum GamePiece {
    NONE,
    CUBE,
    CONE,
  }

  public enum TargetLevel {
    NONE,
    LOW,
    MID,
    HIGH,
  }

  public enum TargetCol {
    NONE,
    LEFT,
    MID,
    RIGHT,
  }
}
