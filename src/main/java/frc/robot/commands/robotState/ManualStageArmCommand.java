package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem.RobotState;
import frc.robot.subsystems.shoulder.MinimalShoulderSubsystem;

public class ManualStageArmCommand extends CommandBase {
  private MinimalRobotStateSubsystem robotStateSubsystem;

  public ManualStageArmCommand(
      MinimalRobotStateSubsystem robotStateSubsystem, MinimalShoulderSubsystem shoulderSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
    addRequirements(shoulderSubsystem);
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toManualScore();
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.getRobotState() == RobotState.STOW;
  }
}
