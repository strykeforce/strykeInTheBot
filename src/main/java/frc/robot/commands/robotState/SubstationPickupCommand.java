package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hand.HandSubsystem;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem.GamePiece;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem.RobotState;
import frc.robot.subsystems.shoulder.MinimalShoulderSubsystem;

public class SubstationPickupCommand extends CommandBase {
  private MinimalRobotStateSubsystem robotStateSubsystem;

  public SubstationPickupCommand(
      MinimalRobotStateSubsystem robotStateSubsystem,
      MinimalShoulderSubsystem shoulderSubsystem,
      HandSubsystem handSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;

    addRequirements(shoulderSubsystem, handSubsystem);
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toManualSubstation(GamePiece.CUBE);
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.getRobotState() == RobotState.STOW;
  }
}
