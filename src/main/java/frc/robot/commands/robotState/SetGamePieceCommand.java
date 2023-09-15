package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.GamePiece;

public class SetGamePieceCommand extends InstantCommand {
  private RobotStateSubsystem robotStateSubsystem;
  private GamePiece gamePiece;

  public SetGamePieceCommand(RobotStateSubsystem robotStateSubsystem, GamePiece gamePiece) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.gamePiece = gamePiece;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.setTargetPiece(gamePiece);
  }
}