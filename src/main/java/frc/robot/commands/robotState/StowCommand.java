package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem.RobotState;
import frc.robot.subsystems.shoulder.MinimalShoulderSubsystem;

public class StowCommand extends CommandBase {
  private MinimalRobotStateSubsystem robotStateSubsystem;

  public StowCommand(
      MinimalRobotStateSubsystem robotStateSubsystem, MinimalShoulderSubsystem shoulderSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
    addRequirements(shoulderSubsystem);
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toStow();
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.getRobotState() == RobotState.STOW;
  }
}
