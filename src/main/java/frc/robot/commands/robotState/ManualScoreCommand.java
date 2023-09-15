package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.hand.HandSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.RobotState;

public class ManualScoreCommand extends CommandBase {
  private RobotStateSubsystem robotStateSubsystem;

  public ManualScoreCommand(
      RobotStateSubsystem robotStateSubsystem,
      ArmSubsystem armSubsystem,
      HandSubsystem handSubsystem) {
    addRequirements(armSubsystem, handSubsystem);
    this.robotStateSubsystem = robotStateSubsystem;
  }

  @Override
  public void initialize() {
    robotStateSubsystem.toManualScore();
  }

  @Override
  public boolean isFinished() {
    return robotStateSubsystem.getCurrRobotState() == RobotState.STOW
        || robotStateSubsystem.getCurrRobotState() == RobotState.MANUAL_SCORE
        || robotStateSubsystem.getCurrRobotState() == RobotState.MANUAL_SHELF;
  }
}