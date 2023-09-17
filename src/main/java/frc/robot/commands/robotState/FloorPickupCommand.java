package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem.GamePiece;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem.RobotState;

public class FloorPickupCommand extends CommandBase {
  private MinimalRobotStateSubsystem robotStateSubsystem;
  private GamePiece gamePiece;

  public FloorPickupCommand(MinimalRobotStateSubsystem robotStateSubsystem, GamePiece gamePiece) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.gamePiece = gamePiece;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toFloorPickup(gamePiece);
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.getRobotState() == RobotState.STOW;
  }
}
