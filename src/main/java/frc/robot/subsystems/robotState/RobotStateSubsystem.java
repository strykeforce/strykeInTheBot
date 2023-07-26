package frc.robot.subsystems.robotState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class RobotStateSubsystem extends SubsystemBase {
  private DriveSubsystem driveSubsystem;
  // private ArmSubsystem armSubsystem;
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

  public RobotStateSubsystem(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
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
    // FIXME needs correct calculation to get whether the robot should goto the shelf cube or cone
    // position based on gyro rotation
    if (isBlueAlliance()) {
      return ((driveSubsystem.getGyroAngle().getRadians() > 0)
          && (driveSubsystem.getGyroAngle().getRadians() < Math.PI));
    } else {
      return ((driveSubsystem.getGyroAngle().getRadians() > Math.PI)
          && (driveSubsystem.getGyroAngle().getRadians() < Math.PI * 2));
    }
  }

  public boolean isBlueAlliance() {
    return allianceColor == Alliance.Blue;
  }

  public void toStow(RobotState nextState) {
    this.nextRobotState = nextState;
    // hand off
    // arm to stow
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
          // arm to floor cube
          // hand cube speed
          break;
        case CONE:
          if (isConePickupUpright) {
            // arm to upright cone
          } else {
            // arm to tipped cone
          }
          // hand cone speed
          break;
        default:
          // (｢•-•)｢ ʷʱʸ?
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
        // arm to shelf cube
        // hand cube speed
      } else {
        // arm to shelf cone
        // hand cone speed
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
        // arm to shelf cube
        // hand cube speed
      } else {
        // arm to shelf cone
        // hand cone speed
      }
      setRobotStateLogged(RobotState.TO_MANUAL_SHELF);
    } else {
      toStow(RobotState.MANUAL_SHELF);
    }
  }

  public void toAutoScore() {
    logger.info("starting auto score");

    toStow(RobotState.TO_AUTO_SCORE);
  }

  public void toManualScore() {
    logger.info("starting manual score");

    toStow(RobotState.TO_MANUAL_SCORE);
  }

  public void toAutobalance() {
    logger.info("starting autobalance");

    setRobotStateLogged(RobotState.TO_AUTOBALANCE);
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
          default:
            break;
        }
        break;
      case FLOOR_PICKUP:
        // wait for beam break
        currentPiece = targetPiece;
        toStow();
        break;
      case AUTO_SHELF:
        // wait to drive to correct spot
        currentPiece = targetPiece;
        currShelfPoseX = driveSubsystem.getPoseMeters().getX();
        break;
      case MANUAL_SHELF:
        // wait for beam break
        // hand to holding speed
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
      case AUTOBALANCE:
        break;
      case TO_STOW:
        // wait for arm to finish
        setRobotStateLogged(RobotState.STOW);
        break;
      case TO_FLOOR_PICKUP:
        // wait for arm to finish
        // turn on rollers
        setRobotStateLogged(RobotState.FLOOR_PICKUP);
        break;
      case TO_AUTO_SHELF:
        // wait for arm to finish
        // driveSubsystem.driveToPose();
        setRobotStateLogged(RobotState.AUTO_SHELF);
        break;
      case TO_MANUAL_SHELF:
        // wait for arm to finish
        setRobotStateLogged(RobotState.MANUAL_SHELF);
        break;
      case TO_AUTO_SCORE:
        // continue to TO_MANUAL_SCORE
      case TO_MANUAL_SCORE:
        switch (targetLevel) {
          case LOW:
            // arm to low
            break;
          case MID:
            // arm to low
            break;
          case HIGH:
            // arm to low
            break;
          default:
            break;
        }
        if (currRobotState == RobotState.TO_AUTO_SCORE) {
          switch (targetCol) {
            case LEFT:
              break;
            case MID:
              break;
            case RIGHT:
              break;
            default:
              break;
          }
        }
        break;
      case TO_AUTOBALANCE:
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
