package frc.robot.subsystems.robotState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.Arm.ArmSubsystem;
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
    }// else {
    //   toStow(RobotState.FLOOR_PICKUP);
    // }
  }

  public void toFloorPickup() {
    toFloorPickup(targetPiece);
  }

  public void toAutoShelf() {
    logger.info("starting auto shelf");

    if (isStowed()) {
      if (isShelfPieceCube()) {
        armSubsystem.shelf(GamePiece.CUBE);
        handSubsystem.grabPiece();
      } else {
        armSubsystem.shelf(GamePiece.CONE);
        handSubsystem.grabPiece();
      }
      setRobotStateLogged(RobotState.TO_AUTO_SHELF);
    }// else {
    //   toStow(RobotState.AUTO_SHELF);
    // }
  }

  public void toManualShelf() {
    logger.info("starting manual shelf");

    if (isStowed()) {
      if (isShelfPieceCube()) {
        armSubsystem.shelf(GamePiece.CUBE);
        handSubsystem.grabPiece();
      } else {
        armSubsystem.shelf(GamePiece.CONE);
        handSubsystem.grabPiece();
      }
      setRobotStateLogged(RobotState.TO_MANUAL_SHELF);
    }// else {
    //   toStow(RobotState.MANUAL_SHELF);
    // }
  }

  public void toAutoScore() {
    logger.info("starting auto score");

    if (isStowed()) {
      switch (currentPiece) {
        case CUBE:
          switch (targetLevel) {
            case LOW:
              armSubsystem.low(GamePiece.CUBE);
              break;
            case MID:
              armSubsystem.mid(GamePiece.CUBE);
              break;
            case HIGH:
              armSubsystem.high(GamePiece.CUBE);
              break;
            default:
              break;
          }
          break;
        case CONE: 
          switch (targetLevel) {
            case LOW:
              armSubsystem.low(GamePiece.CONE);
              break;
            case MID:
              armSubsystem.mid(GamePiece.CONE);
              break;
            case HIGH:
              armSubsystem.high(GamePiece.CONE);
              break;
            default:
              break;
          }
          break;
        case NONE:
          break;
      }
      setRobotStateLogged(RobotState.TO_AUTO_SCORE);
    }// else {
    //   toStow(RobotState.AUTO_SCORE);
    // }
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

  public void toAutobalance() {
    logger.info("starting autobalance");

    if (isStowed()) {
      setRobotStateLogged(RobotState.TO_AUTOBALANCE);
    } else {
      toStow(RobotState.AUTOBALANCE);
    }
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
            toAutobalance();
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
        // hand to holding speed
        currentPiece = targetPiece;
        currShelfPoseX = driveSubsystem.getPoseMeters().getX();
        // arm to shelf wait pos?
        setRobotStateLogged(RobotState.SHELF_WAIT);
        break;
      case MANUAL_SHELF:
        // wait for beam break
        // hand to holding speed
        currentPiece = targetPiece;
        currShelfPoseX = driveSubsystem.getPoseMeters().getX();
        // arm to shelf wait pos?
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
        break;
      case TO_STOW:
        if (armSubsystem.isFinished()){
          setRobotStateLogged(RobotState.STOW);
        }
        break;
      case TO_FLOOR_PICKUP:
        if (armSubsystem.isFinished()){
          handSubsystem.grabPiece();
          setRobotStateLogged(RobotState.FLOOR_PICKUP);
        }
        break;
      case TO_AUTO_SHELF:
        if (armSubsystem.isFinished()){
          // driveSubsystem.driveToPose();
          setRobotStateLogged(RobotState.AUTO_SHELF);
        }
        break;
      case TO_MANUAL_SHELF:
        if (armSubsystem.isFinished()){
          setRobotStateLogged(RobotState.MANUAL_SHELF);
        }
        break;
      case TO_AUTO_SCORE:
        if (armSubsystem.isFinished()){
          // drive to pos
          setRobotStateLogged(RobotState.AUTO_SCORE);
        }
        break;
      case TO_MANUAL_SCORE:
        if (armSubsystem.isFinished()){
          setRobotStateLogged(RobotState.MANUAL_SCORE);
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
