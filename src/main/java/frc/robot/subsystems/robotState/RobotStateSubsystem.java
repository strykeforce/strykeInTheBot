package frc.robot.subsystems.robotState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotStateSubsystem extends SubsystemBase {
  private RobotStates robotState = RobotStates.STOW;

  public RobotStateSubsystem() {}

  public enum RobotStates {
    STOW,
    FLOOR,
    AUTO_SHELF,
    MANUAL_SHELF,
    AUTO_SCORE,
    MANUAL_SCORE,
    AUTOBALANCE,
  }
}
