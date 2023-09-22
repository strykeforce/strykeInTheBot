package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem.TargetLevel;

public class SetTargetLevelCommand extends InstantCommand {
  private MinimalRobotStateSubsystem robotStateSubsystem;
  private TargetLevel targetLevel;

  public SetTargetLevelCommand(
      MinimalRobotStateSubsystem robotStateSubsystem, TargetLevel targetLevel) {
    this.robotStateSubsystem = robotStateSubsystem;
    this.targetLevel = targetLevel;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.setTargetLevel(targetLevel);
  }
}
