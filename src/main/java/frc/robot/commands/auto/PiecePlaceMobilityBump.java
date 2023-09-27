package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.ZeroGyroCommand;
import frc.robot.commands.robotState.ClearGamepieceCommand;
import frc.robot.commands.robotState.ManualScoreCommand;
import frc.robot.commands.robotState.ReleaseGamePieceCommand;
import frc.robot.commands.robotState.SetGamePieceCommand;
import frc.robot.commands.robotState.SetTargetLevelCommand;
import frc.robot.commands.shoulder.ZeroShoulderCommand;
// import frc.robot.commands.vision.SetVisionUpdateCommand; // FIXME currently no visionSubsystem
// exists
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.hand.HandSubsystem;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem.GamePiece;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem.TargetLevel;
// import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.shoulder.MinimalShoulderSubsystem;

public class PiecePlaceMobilityBump extends SequentialCommandGroup implements AutoCommandInterface {

  DriveAutonCommand firstPath;
  DriveAutonCommand secondPath;
  DriveAutonCommand thirdPath;
  DriveAutonCommand fourthPath;
  DriveAutonCommand fallbackPath;
  DriveAutonCommand fallbackPath2;
  private boolean hasGenerated = false;
  private Alliance alliance = Alliance.Invalid;
  private MinimalRobotStateSubsystem robotStateSubsystem;

  public PiecePlaceMobilityBump(
      DriveSubsystem driveSubsystem,
      MinimalRobotStateSubsystem robotStateSubsystem,
      MinimalShoulderSubsystem shoulderSubsystem,
      HandSubsystem handSubsystem,
      //   VisionSubsystem visionSubsystem,
      String pathOne) {
    firstPath = new DriveAutonCommand(driveSubsystem, pathOne, true, true);
    this.robotStateSubsystem = robotStateSubsystem;

    addCommands(
        new ParallelCommandGroup(
            new ZeroGyroCommand(driveSubsystem),
            new ZeroShoulderCommand(shoulderSubsystem),
            new SetGamePieceCommand(robotStateSubsystem, GamePiece.CUBE),
            new SetTargetLevelCommand(robotStateSubsystem, TargetLevel.MID),
            new AutoHoldCubeCommand(handSubsystem)
            // ,new SetVisionUpdateCommand(driveSubsystem, false)
            ),
        new ManualScoreCommand(robotStateSubsystem, shoulderSubsystem, handSubsystem),
        new ReleaseGamePieceCommand(robotStateSubsystem, handSubsystem),
        firstPath,
        new ParallelCommandGroup(new ClearGamepieceCommand(robotStateSubsystem))
        // ,new SetVisionUpdateCommand(driveSubsystem, true)
        );
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
