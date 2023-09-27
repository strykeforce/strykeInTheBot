package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hand.HandSubsystem;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem.RobotState;
import frc.robot.subsystems.shoulder.MinimalShoulderSubsystem;

public class ManualScoreCommand extends CommandBase {
  private MinimalRobotStateSubsystem robotStateSubsystem;

  public ManualScoreCommand(
      MinimalRobotStateSubsystem robotStateSubsystem,
      MinimalShoulderSubsystem shoulderSubsystem,
      HandSubsystem handSubsystem) {
    addRequirements(shoulderSubsystem, handSubsystem);
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toManualScore();
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.getCurrRobotState() == RobotState.MANUAL_SCORE
        || robotStateSubsystem.getCurrRobotState() == RobotState.MANUAL_SUBSTATION;
  }
}
