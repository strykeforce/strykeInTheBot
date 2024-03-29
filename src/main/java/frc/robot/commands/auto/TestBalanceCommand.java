package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AutoBalanceCommand;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.XLockCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.hand.HandSubsystem;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem;
import frc.robot.subsystems.shoulder.MinimalShoulderSubsystem;

public class TestBalanceCommand extends SequentialCommandGroup implements AutoCommandInterface {
  DriveAutonCommand firstPath;
  DriveAutonCommand secondPath;
  private boolean hasGenerated = false;
  private Alliance alliance = Alliance.Invalid;
  private MinimalRobotStateSubsystem robotStateSubsystem;

  public TestBalanceCommand(
      DriveSubsystem driveSubsystem,
      MinimalRobotStateSubsystem robotStateSubsystem,
      MinimalShoulderSubsystem shoulderSubsystem,
      HandSubsystem handSubsystem,
      String pathOne,
      String pathTwo) {
    firstPath = new DriveAutonCommand(driveSubsystem, pathOne, false, true);
    this.robotStateSubsystem = robotStateSubsystem;
    addCommands(
        firstPath,
        new AutoBalanceCommand(true, driveSubsystem, robotStateSubsystem),
        new XLockCommand(driveSubsystem));
  }

  public void generateTrajectory() {
    firstPath.generateTrajectory();
    hasGenerated = true;
    alliance = robotStateSubsystem.getAllianceColor();
  }

  public boolean hasGenerated() {
    return hasGenerated && (alliance == robotStateSubsystem.getAllianceColor());
  }
}
