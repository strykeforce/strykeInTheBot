package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class DriveSubsystem {
  public boolean canGetVisionUpdates() {
    return true; // placeholder
  }

  public boolean isAutoDriving() {
    return true; // placeholder
  }

  public Pose2d getPoseMeters() {
    return new Pose2d(); // placeholder
  }

  public Rotation2d getGyroRotation2d() {
    return new Rotation2d(); // placeholder
  }

  public double getVectorSpeed() {
    return 0; // placeholder
  }

  public void updateOdometryWithVision(Pose2d pose, long timestamp) {}

  public void resetOdometryNoLog(Pose2d pose) {}
}
