package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.robotState.ManualScoreCommand;
import frc.robot.commands.robotState.ReleaseGamepieceCommand;
import frc.robot.commands.robotState.SetGamePieceCommand;
import frc.robot.commands.robotState.SetTargetLevelCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.hand.HandSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem.GamePiece;
import frc.robot.subsystems.robotState.RobotStateSubsystem.TargetLevel;

public class DefaultAutoCommand extends SequentialCommandGroup implements AutoCommandInterface {
  private boolean hasGenerated = false;
  private Alliance alliance = Alliance.Invalid;
  private DriveAutonCommand defaultPath;
  private RobotStateSubsystem robotStateSubsystem;

  public DefaultAutoCommand(
      DriveSubsystem driveSubsystem,
      RobotStateSubsystem robotStateSubsystem,
      HandSubsystem handSubsystem,
      ArmSubsystem armSubsystem) {
    defaultPath = new DriveAutonCommand(driveSubsystem, "straightPathX", true, true);
    this.robotStateSubsystem = robotStateSubsystem;

    addCommands(
        new ParallelCommandGroup(
            new SetGamePieceCommand(robotStateSubsystem, GamePiece.CONE),
            new SetTargetLevelCommand(robotStateSubsystem, TargetLevel.HIGH),
            new AutoGrabConeCommand(handSubsystem),
        new ManualScoreCommand(robotStateSubsystem, armSubsystem, handSubsystem),
        new ReleaseGamepieceCommand(handSubsystem, robotStateSubsystem),
        defaultPath));
  }

  public void generateTrajectory() {
    defaultPath.generateTrajectory();
    hasGenerated = true;
    alliance = robotStateSubsystem.getAllianceColor();
  }

  public boolean hasGenerated() {
    return hasGenerated && (alliance == robotStateSubsystem.getAllianceColor());
  }
}