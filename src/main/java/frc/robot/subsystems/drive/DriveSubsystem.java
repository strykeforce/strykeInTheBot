package frc.robot.subsystems.drive;

import java.util.Set;

import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

import edu.wpi.first.math.geometry.Rotation2d;

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
