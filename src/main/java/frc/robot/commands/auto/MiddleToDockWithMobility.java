package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.AutoBalanceCommand;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.XLockCommand;
import frc.robot.commands.drive.ZeroGyroCommand;
import frc.robot.commands.robotState.ClearGamePieceCommand;
import frc.robot.commands.robotState.ManualScoreCommand;
import frc.robot.commands.robotState.ReleaseGamepieceCommand;
import frc.robot.commands.robotState.SetGamePieceCommand;
import frc.robot.commands.robotState.SetTargetLevelCommand;
// import frc.robot.commands.vision.SetVisionUpdateCommand; // FIXME currently no visionSubsystem
// exists
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.hand.HandSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.GamePiece;
import frc.robot.subsystems.robotState.RobotStateSubsystem.TargetLevel;
// import frc.robot.subsystems.VisionSubsystem;

public class MiddleToDockWithMobility extends SequentialCommandGroup
    implements AutoCommandInterface {

  DriveAutonCommand firstPath;
  DriveAutonCommand secondPath;
  DriveAutonCommand thirdPath;
  DriveAutonCommand fourthPath;
  DriveAutonCommand fallbackPath;
  DriveAutonCommand fallbackPath2;
  private boolean hasGenerated = false;
  private Alliance alliance = Alliance.Invalid;
  private RobotStateSubsystem robotStateSubsystem;

  public MiddleToDockWithMobility(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      ArmSubsystem armSubsystem,
      HandSubsystem handSubsystem,
      //   VisionSubsystem visionSubsystem,
      String pathOne,
      String pathTwo) {
    firstPath = new DriveAutonCommand(driveSubsystem, pathOne, false, true);
    secondPath = new DriveAutonCommand(driveSubsystem, pathTwo, true, false);
    this.robotStateSubsystem = robotStateSubsystem;

    addCommands(
        new ParallelCommandGroup(
            new ZeroGyroCommand(driveSubsystem),
            new SetGamePieceCommand(robotStateSubsystem, GamePiece.CONE),
            new SetTargetLevelCommand(robotStateSubsystem, TargetLevel.HIGH),
            new AutoGrabConeCommand(handSubsystem)
            // ,new SetVisionUpdateCommand(driveSubsystem, false)
            ),
        new ManualScoreCommand(robotStateSubsystem, armSubsystem, handSubsystem),
        new ReleaseGamepieceCommand(handSubsystem, robotStateSubsystem),
        new WaitCommand(1.0),
        firstPath,
        new WaitCommand(0.75),
        new ParallelRaceGroup(
            new AutoWaitForMatchTimeCommand(0.1),
            new SequentialCommandGroup(
                secondPath, new AutoBalanceCommand(false, driveSubsystem, robotStateSubsystem))),
        new XLockCommand(driveSubsystem));
    new ParallelCommandGroup(
        new ClearGamePieceCommand(robotStateSubsystem)
        // ,new SetVisionUpdateCommand(driveSubsystem, true)
        );
  }

  public void generateTrajectory() {
    firstPath.generateTrajectory();
    secondPath.generateTrajectory();
    hasGenerated = true;
    alliance = robotStateSubsystem.getAllianceColor();
  }

  public boolean hasGenerated() {
    return hasGenerated && (alliance == robotStateSubsystem.getAllianceColor());
  }
}
