package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hand.HandSubsystem;
import frc.robot.subsystems.hand.HandSubsystem.HandStates;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem.GamePiece;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem.RobotState;
import frc.robot.subsystems.shoulder.MinimalShoulderSubsystem;

public class FloorPickupCommand extends CommandBase {
  private MinimalRobotStateSubsystem robotStateSubsystem;
  private GamePiece gamePiece;
  private HandSubsystem handSubsystem;

  public FloorPickupCommand(
      MinimalRobotStateSubsystem robotStateSubsystem,
      MinimalShoulderSubsystem shoulderSubsystem,
      HandSubsystem handSubsystem,
      GamePiece gamePiece) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.gamePiece = gamePiece;
    this.handSubsystem = handSubsystem;

    addRequirements(shoulderSubsystem);
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toFloorPickup(gamePiece);
  }

  @Override
  public boolean isFinished() {
    return (robotStateSubsystem.getRobotState() == RobotState.STOW
        || handSubsystem.getState() == HandStates.CUBE);
  }
}
