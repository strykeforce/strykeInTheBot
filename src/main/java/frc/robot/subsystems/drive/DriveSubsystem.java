package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Set;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class DriveSubsystem extends MeasurableSubsystem {
  public Rotation2d getGyroAngle() {
    return new Rotation2d();
  }

  @Override
  public Set<Measure> getMeasures() {
    // TODO Auto-generated method stub
    return null;
  }
}
