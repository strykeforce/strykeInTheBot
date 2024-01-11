package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.ZeroGyroCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem;

public class newAuto4Piece extends SequentialCommandGroup implements AutoCommandInterface {

  DriveAutonCommand firstPath;
  DriveAutonCommand secondPath;
  DriveAutonCommand thirdPath;
  DriveAutonCommand fourthPath;
  DriveAutonCommand fallbackPath;
  DriveAutonCommand fallbackPath2;
  private boolean hasGenerated = false;
  private Alliance alliance = Alliance.Invalid;
  private MinimalRobotStateSubsystem robotStateSubsystem;

  public newAuto4Piece(
      DriveSubsystem driveSubsystem,
      MinimalRobotStateSubsystem robotStateSubsystem,
      String pathOne) {
    firstPath = new DriveAutonCommand(driveSubsystem, pathOne, true, true);
    this.robotStateSubsystem = robotStateSubsystem;

    // addCommands(new setAngleOffsetCommand(driveSubsystem, 30.0 * (Math.PI / 180.0)), firstPath);
    // addCommands(new setAngleOffsetCommand(driveSubsystem, 30.0 * (Math.PI / 180.0)), firstPath);
    addCommands(new ZeroGyroCommand(driveSubsystem), firstPath);
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
