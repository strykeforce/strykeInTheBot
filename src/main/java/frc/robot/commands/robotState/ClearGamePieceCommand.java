package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem;

public class ClearGamePieceCommand extends InstantCommand {
  private MinimalRobotStateSubsystem robotStateSubsystem;

  public ClearGamePieceCommand(MinimalRobotStateSubsystem robotStateSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.clearGamePiece();
  }
}
