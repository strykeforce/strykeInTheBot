package frc.robot.commands.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shoulder.MinimalShoulderSubsystem;
import frc.robot.subsystems.shoulder.MinimalShoulderSubsystem.ShoulderStates;

public class ZeroShoulderCommand extends CommandBase {
  private MinimalShoulderSubsystem shoulderSubsystem;

  public ZeroShoulderCommand(MinimalShoulderSubsystem shoulderSubsystem) {
    this.shoulderSubsystem = shoulderSubsystem;
    addRequirements(shoulderSubsystem);
  }

  @Override
  public void initialize() {
    shoulderSubsystem.zero();
  }

  @Override
  public boolean isFinished() {
    return shoulderSubsystem.getState() == ShoulderStates.ZEROED;
  }
}
