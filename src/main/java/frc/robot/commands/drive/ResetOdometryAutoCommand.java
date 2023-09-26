package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ResetOdometryAutoCommand extends InstantCommand {
  DriveSubsystem driveSubsystem;
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
