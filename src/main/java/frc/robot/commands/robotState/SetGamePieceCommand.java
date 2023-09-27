package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem.GamePiece;

public class SetGamePieceCommand extends InstantCommand {
  private MinimalRobotStateSubsystem robotStateSubsystem;
  private GamePiece gamePiece;

  public SetGamePieceCommand(MinimalRobotStateSubsystem robotStateSubsystem, GamePiece gamePiece) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.gamePiece = gamePiece;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.setGamePiece(gamePiece);
  }
}
