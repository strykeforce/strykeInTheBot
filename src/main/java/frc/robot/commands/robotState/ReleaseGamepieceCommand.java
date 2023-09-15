package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hand.HandSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotState;

public class ReleaseGamepieceCommand extends CommandBase {
  private RobotStateSubsystem robotStateSubsystem;
  private HandSubsystem handSubsystem;

  public ReleaseGamepieceCommand(
      HandSubsystem handSubsystem, RobotStateSubsystem robotStateSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.handSubsystem = handSubsystem;
    addRequirements(handSubsystem);
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toReleaseGamepiece();
  }

  @Override
  public boolean isFinished() {
    RobotState currState = robotStateSubsystem.getCurrRobotState();
    return handSubsystem.isFinished()
        && (currState == RobotState.RELEASE_GAMEPIECE
            || currState == RobotState.TO_STOW
            || currState == RobotState.STOW);
  }
}