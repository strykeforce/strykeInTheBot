package frc.robot.subsystems.drive;

import java.util.Set;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.swerve.SwerveDrive;
import org.strykeforce.swerve.TalonSwerveModule;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;
import org.strykeforce.telemetry.TelemetryService;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;

public class DriveSubsystem extends MeasurableSubsystem {
    private static final Logger logger = LoggerFactory.getLogger(DriveSubsystem.class);
    private org.littletonrobotics.junction.Logger advLogger = org.littletonrobotics.junction.Logger.getInstance();
    private final SwerveDrive swerveDrive;


    public DriveSubsystem(TelemetryService telemetryService) {

        swerveDrive = new SwerveDrive(swerveModules);
        swerveDrive.resetGyro();
        swerveDrive.setGyroOffset(Rotation2d.fromDegrees(0));

    }

    @Override
    public Set<Measure> getMeasures() {
        // TODO Auto-generated method stub
        return null;
    }
}
