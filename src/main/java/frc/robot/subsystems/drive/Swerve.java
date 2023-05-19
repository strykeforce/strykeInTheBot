package frc.robot.subsystems.drive;

import org.strykeforce.swerve.PoseEstimatorOdometryStrategy;
import org.strykeforce.swerve.SwerveDrive;
import org.strykeforce.swerve.SwerveModule;
import org.strykeforce.swerve.TalonSwerveModule;
import org.strykeforce.telemetry.TelemetryService;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.VisionConstants;

public class Swerve implements SwerveIO {

    private final SwerveDrive swerveDrive;
    private final HolonomicDriveController holonomicController;
    private final ProfiledPIDController omegaController;
    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController omegaAutoDriveController;
    private final ProfiledPIDController xAutoDriveController;
    private final ProfiledPIDController yAutoDriveController;
    private PoseEstimatorOdometryStrategy odometryStrategy;
    private AHRS ahrs;

    public Swerve(TelemetryService telemetryService) {
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

        for (int i = 0; i < 4; i++) {
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

        for (int i = 0; i < 4; i++) {
            var driveTalon = new TalonFX(i + 14);
            driveTalon.configFactoryDefault(Constants.kTalonConfigTimeout);
            driveTalon.configAllSettings(DriveConstants.getDriveTalonConfig(), Constants.kTalonConfigTimeout);
            driveTalon.enableVoltageCompensation(true);
            driveTalon.setNeutralMode(NeutralMode.Brake);
            telemetryService.register(driveTalon);
        }

        ahrs = new AHRS(SerialPort.Port.kUSB, SerialDataType.kProcessedData, (byte) 200);
        swerveDrive = new SwerveDrive(ahrs, swerveModules);
        swerveDrive.resetGyro();
        swerveDrive.setGyroOffset(Rotation2d.fromDegrees(0));

        // Setup Holonomic Controller
        omegaAutoDriveController =
        new ProfiledPIDController(
            DriveConstants.kPOmega,
            DriveConstants.kIOmega,
            DriveConstants.kDOmega,
            new TrapezoidProfile.Constraints(
                DriveConstants.kMaxOmega, DriveConstants.kMaxAccelOmega));
        omegaAutoDriveController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));

        xAutoDriveController =
            new ProfiledPIDController(
                DriveConstants.kPAutoDrive,
                DriveConstants.kIAutoDrive,
                DriveConstants.kDAutoDrive,
                new TrapezoidProfile.Constraints(
                    DriveConstants.kAutoDriveMaxVelocity, DriveConstants.kAutoDriveMaxAccel));
        // xAutoDriveController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));

        yAutoDriveController =
            new ProfiledPIDController(
                DriveConstants.kPAutoDrive,
                DriveConstants.kIAutoDrive,
                DriveConstants.kDAutoDrive,
                new TrapezoidProfile.Constraints(
                    DriveConstants.kAutoDriveMaxVelocity, DriveConstants.kAutoDriveMaxAccel));
        // yAutoDriveController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));

        omegaController =
            new ProfiledPIDController(
                DriveConstants.kPOmega,
                DriveConstants.kIOmega,
                DriveConstants.kDOmega,
                new TrapezoidProfile.Constraints(
                    DriveConstants.kMaxOmega, DriveConstants.kMaxAccelOmega));
        omegaController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));

        xController =
            new PIDController(
                DriveConstants.kPHolonomic, DriveConstants.kIHolonomic, DriveConstants.kDHolonomic);
        // xController.setIntegratorRange(DriveConstants.kIMin, DriveConstants.kIMax);
        yController =
            new PIDController(
                DriveConstants.kPHolonomic, DriveConstants.kIHolonomic, DriveConstants.kDHolonomic);
        // yController.setIntegratorRange(DriveConstants.kIMin, DriveConstants.kIMax);
        holonomicController = new HolonomicDriveController(xController, yController, omegaController);
        // Disabling the holonomic controller makes the robot directly follow the trajectory output (no
        // closing the loop on x,y,theta errors)
        holonomicController.setEnabled(true);
        odometryStrategy =
            new PoseEstimatorOdometryStrategy(
                swerveDrive.getHeading(),
                new Pose2d(),
                swerveDrive.getKinematics(),
                VisionConstants.kStateStdDevs,
                VisionConstants.kLocalMeasurementStdDevs,
                VisionConstants.kVisionMeasurementStdDevs,
                getSwerveModulePositions());
        swerveDrive.setOdometry(odometryStrategy);
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    public SwerveModule[] getSwerveModules() {
        return swerveDrive.getSwerveModules();
      }
    
    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModule[] swerveModules = getSwerveModules();
        SwerveModulePosition[] temp = {null, null, null, null};
        for (int i = 0; i < 4; ++i) {
          temp[i] = swerveModules[i].getPosition();
        }
        return temp;
      }

    @Override
    public void updateInputs(SwerveIOInputs inputs) {
        // TODO Auto-generated method stub
        SwerveIO.super.updateInputs(inputs);
    }

    @Override
    public void registerWith(TelemetryService telemetryService) {
        
    }
}
