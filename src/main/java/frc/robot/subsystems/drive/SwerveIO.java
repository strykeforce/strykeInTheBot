package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

public interface SwerveIO {

  @AutoLog
  public static class SwerveIOInputs {
    public Pose2d odometry = new Pose2d();
    public double gyroRotation = 0.0;
    public double odometryRotation2D = 0.0;
  }

  public default void updateInputs(SwerveIOInputs inputs) {}

  public default void registerWith(TelemetryService telemetryService) {}
}
