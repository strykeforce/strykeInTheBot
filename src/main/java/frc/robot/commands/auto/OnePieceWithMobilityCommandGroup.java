package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.robotState.ManualScoreCommand;
import frc.robot.commands.robotState.ReleaseGamepieceCommand;
import frc.robot.commands.robotState.SetGamePieceCommand;
import frc.robot.commands.robotState.SetTargetLevelCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.hand.HandSubsystem;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem.GamePiece;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem.TargetLevel;
import frc.robot.subsystems.shoulder.MinimalShoulderSubsystem;

public class OnePieceWithMobilityCommandGroup extends SequentialCommandGroup
    implements AutoCommandInterface {
  DriveAutonCommand firstPath;
  DriveAutonCommand secondPath;
  private boolean hasGenerated = false;
  private Alliance alliance = Alliance.Invalid;
  private MinimalRobotStateSubsystem robotStateSubsystem;

  public void TwoPieceAutoPlacePathCommandGroup(
      DriveSubsystem driveSubsystem,
      MinimalRobotStateSubsystem robotStateSubsystem,
      MinimalShoulderSubsystem shoulderSubsystem,
      HandSubsystem handSubsystem,
      String pathOne) {
    firstPath = new DriveAutonCommand(driveSubsystem, pathOne, true, true);
    this.robotStateSubsystem = robotStateSubsystem;

    addCommands(
        new ParallelCommandGroup(
            new SetGamePieceCommand(robotStateSubsystem, GamePiece.CUBE),
            new SetTargetLevelCommand(robotStateSubsystem, TargetLevel.HIGH)),
        new ManualScoreCommand(robotStateSubsystem, shoulderSubsystem, handSubsystem),
        new ReleaseGamepieceCommand(robotStateSubsystem, handSubsystem),
        firstPath);
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
