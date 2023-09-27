package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.hand.HandSubsystem;

public class AutoHoldCubeCommand extends InstantCommand {
  private HandSubsystem handSubsystem;

  public AutoHoldCubeCommand(HandSubsystem handSubsystem) {
    this.handSubsystem = handSubsystem;

    addRequirements(handSubsystem);
  }

  @Override
  public void initialize() {
    handSubsystem.autoHoldCube();
  }
}
