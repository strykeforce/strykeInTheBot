package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem;

public class ManualStageArmCommand extends CommandBase {
  private MinimalRobotStateSubsystem robotStateSubsystem;

  public ManualStageArmCommand(MinimalRobotStateSubsystem robotStateSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toManualScore();
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return super.isFinished();
  }
}
