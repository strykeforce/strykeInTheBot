package frc.robot.subsystems.drive;

import org.strykeforce.swerve.TalonSwerveModule;
import org.strykeforce.telemetry.TelemetryService;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;

public class Modules implements ModulesIO {
    public Modules() {
        var moduleBuilder = new TalonSwerveModule.Builder()
                .driveGearRatio(DriveConstants.kDriveGearRatio)
                .wheelDiameterInches(DriveConstants.kWheelDiameterInches)
                .driveMaximumMetersPerSecond(DriveConstants.kMaxSpeedMetersPerSecond);

        TalonSwerveModule[] swerveModules = new TalonSwerveModule[4];
        Translation2d[] wheelLocations = DriveConstants.getWheelLocationMeters();

        for (int i = 0; i < 4; i++) {
            var azimuthTalon = new TalonSRX(i);
            azimuthTalon.configFactoryDefault(Constants.kTalonConfigTimeout);
            azimuthTalon.configAllSettings(DriveConstants.getAzimuthTalonConfig(), Constants.kTalonConfigTimeout);
            azimuthTalon.enableCurrentLimit(true);
            azimuthTalon.enableVoltageCompensation(true);
            azimuthTalon.setNeutralMode(NeutralMode.Coast);
            swerveModules[i] = moduleBuilder
                    .azimuthTalon(azimuthTalon)
                    .wheelLocationMeters(wheelLocations[i])
                    .build();

            swerveModules[i].loadAndSetAzimuthZeroReference();
            telemetryService.register(azimuthTalon);
        }

        for (int i = 0; i < 8; i++) {
            var driveTalon = new TalonFX(i + 10);
            driveTalon.configFactoryDefault(Constants.kTalonConfigTimeout);
            driveTalon.configAllSettings(DriveConstants.getDriveTalonConfig(), Constants.kTalonConfigTimeout);
            driveTalon.enableVoltageCompensation(true);
            driveTalon.setNeutralMode(NeutralMode.Brake);
            swerveModules[i] = moduleBuilder
                    .driveTalon(driveTalon)
                    .wheelLocationMeters(wheelLocations[i])
                    .build();
            telemetryService.register(driveTalon);
        }
    }


    @Override
    public void registerWith(TelemetryService telemetryService) {
        telemetryService.register();
    }
}
