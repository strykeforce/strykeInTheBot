package frc.robot.subsystems.robotState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class RobotStateSubsystem extends SubsystemBase {
  private DriveSubsystem driveSubsystem;
  // private ArmSubsystem armSubsystem;
  private Alliance allianceColor = DriverStation.getAlliance();
  private Logger logger = LoggerFactory.getLogger(RobotStateSubsystem.class);
  private RobotState robotState = RobotState.STOW;
  private RobotState nextState = RobotState.STOW;
  private GamePiece currentPiece = GamePiece.NONE;
  private GamePiece targetPiece = GamePiece.NONE;
  private TargetLevel targetLevel = TargetLevel.NONE;
  private TargetCol targetCol = TargetCol.NONE;
  private boolean isConePickupUpright = true;

  public RobotStateSubsystem(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
  }

  public void setAllianceColor(Alliance alliance) {
    // logger.info("Changing to {}", alliance);
    this.allianceColor = alliance;
  }

  public Alliance getAllianceColor() {
    return allianceColor;
  }

  public void toStow(RobotState nextState) {
    this.nextState = nextState;
    if (robotState != RobotState.STOW) {
      robotState = RobotState.TO_STOW;
    }
  }

  public void toStow() {
    toStow(RobotState.STOW);
  }

  public void toFloorPickup(GamePiece targetPiece) {
    this.targetPiece = targetPiece;
    toStow(RobotState.TO_FLOOR_PICKUP);
  }

  public void toShelfPickup() {
    boolean logik = true;
    if (logik) { // FIXME: logik
      robotState = RobotState.TO_AUTO_SHELF;
    } else {
      robotState = RobotState.TO_MANUAL_SHELF;
    }
    if (driveSubsystem.getGyroAngle().getRadians()
        > 0) { // FIXME: change calulation and also use alliance color
      targetPiece = GamePiece.CONE;
    } else {
      targetPiece = GamePiece.CUBE;
    }
  }

  public void toScore() {
    robotState = RobotState.TO_SCORE;
  }

  public void toAutobalance() {
    robotState = RobotState.TO_AUTOBALANCE;
  }

  @Override
  public void periodic() {
    switch (robotState) {
      case STOW:
        // basically do nothing
        robotState = nextState;
        break;
      case FLOOR_PICKUP:
        // wait for beam break
        currentPiece = targetPiece;
        toStow();
        break;
      case AUTO_SHELF:
        break;
      case MANUAL_SHELF:
        break;
      case SCORE:
        break;
      case AUTOBALANCE:
        break;
      case TO_STOW:
        // arm to stow
        // hand rollers off
        robotState = RobotState.STOW;
        break;
      case TO_FLOOR_PICKUP:
        if (targetPiece == GamePiece.CONE) {
          if (isConePickupUpright) {}
        } else if (targetPiece == GamePiece.CUBE) {
          // arm to cube
        }
        // once arm is done
        // turn on rollers
        robotState = RobotState.FLOOR_PICKUP;
        break;
      case TO_AUTO_SHELF:
        break;
      case TO_MANUAL_SHELF:
        break;
      case TO_SCORE:
        //
        break;
      case TO_AUTOBALANCE:
        break;
      default:
        logger.warn("{} is an invalid robot state!", robotState);
        break;
    }
  }

  public enum RobotState {
    STOW,
    FLOOR_PICKUP,
    AUTO_SHELF,
    MANUAL_SHELF,
    SCORE,
    AUTOBALANCE,
    TO_STOW,
    TO_FLOOR_PICKUP,
    TO_AUTO_SHELF,
    TO_MANUAL_SHELF,
    TO_SCORE,
    TO_AUTOBALANCE,
  }

  public enum GamePiece {
    CONE,
    CUBE,
    NONE
  }

  public enum TargetLevel {
    NONE,
    LOW,
    MID,
    HIGH
  }

  public enum TargetCol {
    NONE,
    LEFT,
    MID,
    RIGHT
  }
}
