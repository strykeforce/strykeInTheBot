package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.RobotState.RobotStateSubsystem;

public class ResetOdometryCommand extends InstantCommand {
  DriveSubsystem driveSubsystem;
  RobotStateSubsystem robotStateSubsystem;

  public ResetOdometryCommand(
      DriveSubsystem driveSubsystem, RobotStateSubsystem robotStateSubsystem) {
    addRequirements(driveSubsystem);
    this.robotStateSubsystem = robotStateSubsystem;
    this.driveSubsystem = driveSubsystem;
  }

  @Override
  public void initialize() {

    if (robotStateSubsystem.getAllianceColor() == Alliance.Blue)
      driveSubsystem.resetOdometry(DriveConstants.kOdometryZeroPosBlue);
    else driveSubsystem.resetOdometry(DriveConstants.kOdometryZeroPosRed);
  }
}
