package frc.robot.subsystems.robotState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotStateSubsystem extends SubsystemBase {
  // private ArmSubsystem armSubsystem;
  private RobotStates robotState = RobotStates.STOW;

  public RobotStateSubsystem() {}

  public void toStow() {
    robotState = RobotStates.TO_STOW;
  }

  public void toFloorPickup() {
    robotState = RobotStates.TO_FLOOR_PICKUP;
  }

  public void toShelfPickup(boolean isAuto) {
    if (isAuto) { // FIXME: maybe do logic inside the function (eg. which direction we are facing)
      robotState = RobotStates.TO_AUTO_SHELF;
    } else {
      robotState = RobotStates.TO_MANUAL_SHELF;
    }
  }

  public void toScore() {
    robotState = RobotStates.TO_SCORE;
  }

  public void toAutobalance() {
    robotState = RobotStates.TO_AUTOBALANCE;
  }

  @Override
  public void periodic() {
    switch (robotState) {
      case STOW:
        break;
      case FLOOR_PICKUP:
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
        break;
      case TO_FLOOR_PICKUP:
        break;
      case TO_AUTO_SHELF:
        break;
      case TO_MANUAL_SHELF:
        break;
      case TO_SCORE:
        break;
      case TO_AUTOBALANCE:
        break;
      default:
        break;
    }
  }

  public enum RobotStates {
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
}
