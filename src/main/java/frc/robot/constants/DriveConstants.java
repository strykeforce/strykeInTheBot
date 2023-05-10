package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.geometry.Translation2d;

public class DriveConstants {
    public static final int kTalonConfigTimeout = 10; // ms
    
    public static final double kRobotLength = 0;
    public static final double kRobotWidth = 0;
    
    public static final double kDriveGearRatio = 0;
    public static final double kWheelDiameterInches = 0;
    public static final double kMaxSpeedMetersPerSecond = 0;


    public static Translation2d[] getWheelLocationMeters() {
        final double x = kRobotLength / 2.0; // front-back, was ROBOT_LENGTH
        final double y = kRobotWidth / 2.0; // left-right, was ROBOT_WIDTH
        Translation2d[] locs = new Translation2d[4];
        locs[0] = new Translation2d(x, y); // left front
        locs[1] = new Translation2d(x, -y); // right front
        locs[2] = new Translation2d(-x, y); // left rear
        locs[3] = new Translation2d(-x, -y); // right rear
        return locs;
    }

    public static TalonSRXConfiguration getAzimuthTalonConfig() {
        // constructor sets encoder to Quad/CTRE_MagEncoder_Relative
        TalonSRXConfiguration azimuthConfig = new TalonSRXConfiguration();
  
        azimuthConfig.primaryPID.selectedFeedbackCoefficient = 1.0;
        azimuthConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.None;
  
        azimuthConfig.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
        azimuthConfig.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;
  
        azimuthConfig.continuousCurrentLimit = 10;
        azimuthConfig.peakCurrentDuration = 0;
        azimuthConfig.peakCurrentLimit = 0;

        azimuthConfig.slot0.kP = 10.0;
        azimuthConfig.slot0.kI = 0.0;
        azimuthConfig.slot0.kD = 100.0;
        azimuthConfig.slot0.kF = 1.0;
        azimuthConfig.slot0.integralZone = 0;
        azimuthConfig.slot0.allowableClosedloopError = 0;
        azimuthConfig.slot0.maxIntegralAccumulator = 0;
        
        azimuthConfig.motionCruiseVelocity = 800;
        azimuthConfig.motionAcceleration = 10_000;
        azimuthConfig.velocityMeasurementWindow = 64;
        azimuthConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
        azimuthConfig.voltageCompSaturation = 12;
        azimuthConfig.voltageMeasurementFilter = 32;
        azimuthConfig.neutralDeadband = 0.04;
        return azimuthConfig;
      }

        // Drive Falcon Config
        public static TalonFXConfiguration getDriveTalonConfig() {
            TalonFXConfiguration driveConfig = new TalonFXConfiguration();
            driveConfig.supplyCurrLimit.currentLimit = 40;
            driveConfig.supplyCurrLimit.triggerThresholdCurrent = 45;
            driveConfig.supplyCurrLimit.triggerThresholdTime = 1.0;
            driveConfig.supplyCurrLimit.enable = true;
            driveConfig.statorCurrLimit.enable = false;
            driveConfig.slot0.kP = 0.16; // 0.16
            driveConfig.slot0.kI = 0.0002;
            driveConfig.slot0.kD = 0.000;
            driveConfig.slot0.kF = 0.047;
            driveConfig.slot0.integralZone = 500;
            driveConfig.slot0.maxIntegralAccumulator = 150_000;
            driveConfig.slot0.allowableClosedloopError = 0;
            driveConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
            driveConfig.velocityMeasurementWindow = 64;
            driveConfig.voltageCompSaturation = 12;
            driveConfig.neutralDeadband = 0.01;
            driveConfig.voltageMeasurementFilter = 32;
            return driveConfig;
          }

}
