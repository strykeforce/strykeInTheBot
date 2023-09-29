package frc.robot.commands.hand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hand.HandSubsystem;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.GamePiece;

public class HandReleaseGamepieceCommand extends CommandBase {
  private HandSubsystem handSubsystem;
  private MinimalRobotStateSubsystem robotStateSubsystem;

  public HandReleaseGamepieceCommand(
      HandSubsystem handSubsystem, MinimalRobotStateSubsystem minimalRobotStateSubsystem) {
    this.handSubsystem = handSubsystem;
    this.robotStateSubsystem = robotStateSubsystem;
    addRequirements(handSubsystem);
  }

  @Override
  public void initialize() {
    handSubsystem.ejectPiece(GamePiece.CUBE, RobotStateSubsystem.TargetLevel.MID);
    robotStateSubsystem.clearGamePiece();
  }
}
