package frc.robot.constants;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

public class DriveConstants {
  public static final double kFieldLength = 16.54;

  public static final int kTalonConfigTimeout = 10; // ms

  public static final double kRobotLength = 0;
  public static final double kRobotWidth = 0;

  public static final double kDriveGearRatio = 0;
  public static final double kWheelDiameterInches = 0;
  public static final double kMaxSpeedMetersPerSecond = 0;

  public static final double kMaxOmega =
      (kMaxSpeedMetersPerSecond / Math.hypot(kRobotWidth / 2.0, kRobotLength / 2.0))
          / 2.0; // wheel locations below

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

  // Holonomic Controller Constants
  public static final double kPHolonomic = 0.25; // 6 0.25
  public static final double kIHolonomic = 0.0000;
  public static final double kDHolonomic = 0.00; // kPHolonomic/100
  public static final double kIMin = 0.0;
  public static final double kIMax = 0.0;

  public static final double kPOmega = 4.5;
  public static final double kIOmega = 0.0;
  public static final double kDOmega = 0.0;
  //    public static final double kMaxVelOmega = kMaxOmega / 2.0;
  public static final double kMaxAccelOmega = 5.0; // 3.14

  // AUTODRIVE ProfiledPID Constants
  public static final double kPAutoDrive = 3; // 3
  public static final double kIAutoDrive = 0.0000;
  public static final double kDAutoDrive = 0.00; // kPHolonomic/100

  public static final double kAutoDriveMaxVelocity = 2; //
  public static final double kAutoDriveMaxAccel = 2;

  // Auto PickUp Constants
  public static final double kPAutoPickup = 3;
  public static final double kIAutoPickup = 0.0;
  public static final double kDAutoPickup = 0.0;

  public static final double kAutoPickupDriveMaxVel = 1.5;
  public static final double kAutoPickupDriveMaxAccel = 2;

  public static final double kAutoPickupCloseEnough = 0.07;


  // Default safety path constants
  public static final Pose2d startPose2d = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  public static final Pose2d endPose2d = new Pose2d(-1, 0, Rotation2d.fromDegrees(0));

  public static ArrayList<Translation2d> getDefaultInternalWaypoints() {
    ArrayList<Translation2d> waypoints = new ArrayList<>();
    waypoints.add(new Translation2d(-0.5, 0));
    return waypoints;
  }

  public static TrajectoryConfig getDefaultTrajectoryConfig() {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(1, 1);
    trajectoryConfig.setReversed(true);
    trajectoryConfig.setStartVelocity(0.0);
    trajectoryConfig.setEndVelocity(0.0);
    return trajectoryConfig;
  }

  // Azimuth Talon Config
  public static SupplyCurrentLimitConfiguration getAzimuthSupplyCurrentLimit() {
    return new SupplyCurrentLimitConfiguration(true, 10, 15, 0.04);
  }



}
