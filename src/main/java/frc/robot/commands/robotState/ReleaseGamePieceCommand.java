package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hand.HandSubsystem;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem.RobotState;

public class ReleaseGamePieceCommand extends CommandBase {
  private MinimalRobotStateSubsystem robotStateSubsystem;

  public ReleaseGamePieceCommand(
      MinimalRobotStateSubsystem robotStateSubsystem, HandSubsystem handSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
    addRequirements(handSubsystem);
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toReleaseGamepiece();
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.getRobotState() == RobotState.STOW;
  }
}
