package frc.robot.commands.example;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.example.ExampleSubsystem;

public class OpenExampleCommand extends CommandBase {
  private final ExampleSubsystem exampleSubsystem;

  public OpenExampleCommand(ExampleSubsystem exampleSubsystem) {
    this.exampleSubsystem = exampleSubsystem;
    addRequirements(exampleSubsystem);
  }

  @Override
  public void initialize() {
    exampleSubsystem.open();
  }

  @Override
  public boolean isFinished() {
    return exampleSubsystem.isFinished();
  }
}
