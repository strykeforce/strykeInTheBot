package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;

public class ResetOdometryAutoCommand extends InstantCommand {
  DriveSubsystem driveSubsystem;
  RobotStateSubsystem robotStateSubsystem;
  Pose2d pos;

  public ResetOdometryAutoCommand(DriveSubsystem driveSubsystem, Pose2d pos) {
    addRequirements(driveSubsystem);
    this.pos = pos;
    this.driveSubsystem = driveSubsystem;
  }

  @Override
  public void initialize() {
    driveSubsystem.resetOdometry(pos);
  }
}
