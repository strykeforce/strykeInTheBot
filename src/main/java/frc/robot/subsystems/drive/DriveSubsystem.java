package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem;
import frc.robot.subsystems.timestampedpose.TimestampedPose;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Set;
import net.consensys.cava.toml.Toml;
import net.consensys.cava.toml.TomlArray;
import net.consensys.cava.toml.TomlParseResult;
import net.consensys.cava.toml.TomlTable;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.swerve.SwerveDrive;
import org.strykeforce.swerve.SwerveModule;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class DriveSubsystem extends MeasurableSubsystem {
  private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();
  private static final Logger logger = LoggerFactory.getLogger(DriveSubsystem.class);
  private org.littletonrobotics.junction.Logger advLogger =
      org.littletonrobotics.junction.Logger.getInstance();
  private final Swerve swerve;
  private final SwerveDrive swerveDrive;
  private final HolonomicDriveController holonomicController;
  private Alliance allianceColor = DriverStation.getAlliance();
  private MinimalRobotStateSubsystem robotStateSubsystem;

  private final ProfiledPIDController omegaController;
  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController omegaAutoDriveController;
  private final ProfiledPIDController xAutoDriveController;
  private final ProfiledPIDController yAutoDriveController;

  public DriveStates currDriveState = DriveStates.IDLE;

  // Grapher stuff
  private ChassisSpeeds holoContOutput = new ChassisSpeeds();
  private State holoContInput = new State();
  private Rotation2d holoContAngle = new Rotation2d();
  private Double trajectoryActive = 0.0;
  private double[] lastVelocity = new double[3];
  public boolean autoBalanceGyroActive = false;
  private double autoBalanceStableCounts = 0;
  private Timer autoBalanceTimer = new Timer();
  private Timer autoBalancePulseTimer = new Timer();
  private Timer autoBalanceRecoveryTimer = new Timer();
  public boolean autoBalanceReadjust = false;
  public boolean autoBalanceTempStable = false;
  public double tempRoll;
  private double autoBalanceAvgCount = 0;
  private double sumRoll = 0;
  private double avgStartingRoll = 0;
  private boolean yawAdjustmentActive = false;
  private double recoveryStartingPitch;
  private boolean recoveryStartingPitchSet;
  private double autoBalanceGyroDirection = 0;
  private double yawAdjust = 0;
  private boolean isOnAllianceSide;

  private TimestampedPose timestampedPose =
      new TimestampedPose(RobotController.getFPGATime(), new Pose2d());

  public DriveSubsystem() {
    this.swerve = new Swerve();
    this.swerveDrive = swerve.getSwerveDrive();

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
    // xAutoDriveController.enableContinuousInput(Math.toRadians(-180),
    // Math.toRadians(180));

    yAutoDriveController =
        new ProfiledPIDController(
            DriveConstants.kPAutoDrive,
            DriveConstants.kIAutoDrive,
            DriveConstants.kDAutoDrive,
            new TrapezoidProfile.Constraints(
                DriveConstants.kAutoDriveMaxVelocity, DriveConstants.kAutoDriveMaxAccel));
    // yAutoDriveController.enableContinuousInput(Math.toRadians(-180),
    // Math.toRadians(180));

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
    // Disabling the holonomic controller makes the robot directly follow the
    // trajectory output (no
    // closing the loop on x,y,theta errors)
    holonomicController.setEnabled(true);
  }

  // Open-Loop Swerve Movements
  public void drive(double vXmps, double vYmps, double vOmegaRadps) {
    swerveDrive.drive(vXmps, vYmps, vOmegaRadps, true);
  }

  // Closed-Loop (Velocity Controlled) Swerve Movement
  public void move(double vXmps, double vYmps, double vOmegaRadps, boolean isFieldOriented) {
    swerveDrive.move(vXmps, vYmps, vOmegaRadps, isFieldOriented);
  }

  // Holonomic Controller
  public void calculateController(State desiredState, Rotation2d desiredAngle) {
    holoContInput = desiredState;
    holoContAngle = desiredAngle;
    holoContOutput =
        holonomicController.calculate(swerve.getPoseMeters(), desiredState, desiredAngle);
    // logger.info("input: {}, output: {}, angle: {}", holoContInput,
    // holoContOutput, desiredAngle);
    swerveDrive.move(
        holoContOutput.vxMetersPerSecond,
        holoContOutput.vyMetersPerSecond,
        holoContOutput.omegaRadiansPerSecond,
        false);
  }

  public void setRobotStateSubsystem(MinimalRobotStateSubsystem robotStateSubsystem) {
    this.robotStateSubsystem = robotStateSubsystem;
  }

  public void resetOdometry(Pose2d pose) {
    swerve.resetOdometry(pose);
    logger.info("reset odometry with: {}", pose);
  }

  public Pose2d getPoseMeters() {
    return swerve.getPoseMeters();
  }

  public void resetHolonomicController() {
    xController.reset();
    yController.reset();
    omegaController.reset(swerve.getGyroRotation2d().getRadians());
  }

  public Rotation2d getGyroRotation2d() {
    return swerve.getGyroRotation2d();
  }

  public void teleResetGyro() {
    logger.info("Driver Joystick: Reset Gyro");
    double gyroResetDegs = robotStateSubsystem.getAllianceColor() == Alliance.Blue ? 0.0 : 180.0;
    swerve.setGyroOffset(Rotation2d.fromDegrees(gyroResetDegs));
    swerve.resetGyro();
    swerve.resetOdometry(
        new Pose2d(swerve.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(gyroResetDegs)));
  }

  // Make whether a trajectory is currently active obvious on grapher
  public void grapherTrajectoryActive(Boolean active) {
    if (active) trajectoryActive = 1.0;
    else trajectoryActive = 0.0;
  }

  public void setEnableHolo(boolean enabled) {
    holonomicController.setEnabled(enabled);
    logger.info("Holonomic Controller Enabled: {}", enabled);
  }

  private boolean shouldFlip() {
    return allianceColor == Alliance.Red;
  }

  // Field flipping stuff
  public Translation2d apply(Translation2d translation) {
    if (shouldFlip()) {
      return new Translation2d(DriveConstants.kFieldMaxX - translation.getX(), translation.getY());
    } else {
      return translation;
    }
  }

  public Pose2d apply(Pose2d pose) {
    if (shouldFlip()) {
      return new Pose2d(
          DriveConstants.kFieldMaxX - pose.getX(),
          pose.getY(),
          new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
    } else {
      return pose;
    }
  }

  public Rotation2d apply(Rotation2d rotation) {
    logger.info(
        "initial target yaw: {}, cos: {}, sin: {}", rotation, rotation.getCos(), rotation.getSin());
    if (shouldFlip()) {
      return new Rotation2d(-rotation.getCos(), rotation.getSin());
    } else return rotation;
  }

  public ChassisSpeeds getFieldRelSpeed() {
    return swerve.getFieldRelSpeed();
  }

  // Trajectory TOML Parsing
  public PathData generateTrajectory(String trajectoryName) {
    try {
      if (shouldFlip()) {
        logger.info("Flipping path");
      }
      TomlParseResult parseResult =
          Toml.parse(Paths.get("/home/lvuser/deploy/paths/" + trajectoryName + ".toml"));
      logger.info("Generating Trajectory: {}", trajectoryName);
      Pose2d startPose = parsePose2d(parseResult, "start_pose");
      startPose = apply(startPose);
      Pose2d endPose = parsePose2d(parseResult, "end_pose");
      endPose = apply(endPose);
      TomlArray internalPointsToml = parseResult.getArray("internal_points");
      ArrayList<Translation2d> path = new ArrayList<>();
      logger.info("Toml Internal Points Array Size: {}", internalPointsToml.size());

      // Create a table for each internal point and put them into a translation2d
      // waypoint
      for (int i = 0; i < internalPointsToml.size(); i++) {
        TomlTable waypointToml = internalPointsToml.getTable(i);
        Translation2d waypoint =
            new Translation2d(waypointToml.getDouble("x"), waypointToml.getDouble("y"));
        waypoint = apply(waypoint);
        path.add(waypoint);
      }

      // Trajectory Config parsed from toml - any additional constraints would be
      // added here
      TrajectoryConfig trajectoryConfig =
          new TrajectoryConfig(
              parseResult.getDouble("max_velocity"), parseResult.getDouble("max_acceleration"));
      logger.info("max velocity/acceleration worked");
      trajectoryConfig.setReversed(parseResult.getBoolean("is_reversed"));
      trajectoryConfig.setStartVelocity(parseResult.getDouble("start_velocity"));
      logger.info("start velocity worked");
      trajectoryConfig.setEndVelocity(parseResult.getDouble("end_velocity"));
      logger.info("end velocity worked");

      // Yaw degrees is seperate from the trajectoryConfig
      double yawDegrees = parseResult.getDouble("target_yaw");
      logger.info("target yaw worked");
      Rotation2d target_yaw = Rotation2d.fromDegrees(yawDegrees);
      target_yaw = apply(target_yaw);
      logger.info("Yaw is {}", target_yaw);

      // Create a the generated trajectory and return it along with the target yaw
      Trajectory trajectoryGenerated =
          TrajectoryGenerator.generateTrajectory(startPose, path, endPose, trajectoryConfig);

      logger.info("Total Time: {}", trajectoryGenerated.getTotalTimeSeconds());
      return new PathData(target_yaw, trajectoryGenerated);
    } catch (Exception error) {
      logger.error(error.toString());
      logger.error("Path {} not found - Running Default Path", trajectoryName);

      // In the case of an error, set the trajectory to default
      Trajectory trajectoryGenerated =
          TrajectoryGenerator.generateTrajectory(
              DriveConstants.startPose2d,
              DriveConstants.getDefaultInternalWaypoints(),
              DriveConstants.endPose2d,
              DriveConstants.getDefaultTrajectoryConfig());

      return new PathData(swerve.getGyroRotation2d(), trajectoryGenerated);
    }
  }

  private Pose2d parsePose2d(TomlParseResult parseResult, String pose) {
    return new Pose2d(
        parseResult.getTable(pose).getDouble("x"),
        parseResult.getTable(pose).getDouble("y"),
        Rotation2d.fromDegrees(parseResult.getTable(pose).getDouble("angle")));
  }

  public void lockZero() {
    SwerveModule[] swerveModules = swerve.getSwerveModules();
    for (int i = 0; i < 4; i++) {
      swerveModules[i].setAzimuthRotation2d(Rotation2d.fromDegrees(0.0));
    }
  }

  public void xLock() {
    SwerveModule[] swerveModules = swerve.getSwerveModules();
    for (int i = 0; i < 4; i++) {
      if (i == 0 || i == 3) {
        swerveModules[i].setAzimuthRotation2d(Rotation2d.fromDegrees(45.0));
      }

      if (i == 1 || i == 2) {
        swerveModules[i].setAzimuthRotation2d(Rotation2d.fromDegrees(-45.0));
      }
    }
  }

  public void setDriveState(DriveStates driveStates) {
    logger.info("{} -> {}", currDriveState, driveStates);
    currDriveState = driveStates;
  }

  public DriveStates getDriveState() {
    return currDriveState;
  }

  public void autoBalance(boolean isOnAllianceSide) {
    // initializing autobalance variables
    autoBalanceAvgCount = 0;
    sumRoll = 0;
    avgStartingRoll = 0;
    this.isOnAllianceSide = isOnAllianceSide;
    tempRoll = swerve.getGyroPitch();
    logger.info(
        "Starting AutoBalance: tempRoll: {}, Alliance: {}, isOnAllianceSide: {}",
        tempRoll,
        robotStateSubsystem.getAllianceColor(),
        isOnAllianceSide);

    // On alliance side of charge station, drive Positive X
    if (isOnAllianceSide) {
      move(DriveConstants.kAutoBalanceFinalDriveVel, 0, 0, false);
      logger.info("DRIVING POSITIVE LIKE ITS SUPPOSED TO");
    }
    // NOT On alliance side of charge station, drive Negative X
    else move(-DriveConstants.kAutoBalanceFinalDriveVel, 0, 0, false);
    logger.info("{} -> AUTO_BALANCE_EDGE", currDriveState);
    currDriveState = DriveStates.AUTO_BALANCE_EDGE;
  }

  public void pulseAutoBalance(boolean isOnAllianceSide) {
    this.isOnAllianceSide = isOnAllianceSide;
    autoBalancePulseTimer.start();
    autoBalancePulseTimer.reset();
    tempRoll = Math.abs(swerve.getGyroPitch());
    move(0.0, 0.0, 0.0, false);
    logger.info("{} -> AUTO_BALANCE_CHECK", currDriveState);
    currDriveState = DriveStates.AUTO_BALANCE_CHECK;
  }

  public boolean isWithinThresholdAutoBalance(double threshold) {
    return (swerve.getGyroRoll() <= threshold && swerve.getGyroRoll() >= -threshold);
  }

  public boolean isNavxWorking() {
    return swerve.isConnected();
  }

  public void setGyroOffset(Rotation2d angle) {
    swerve.setGyroOffset(angle);
  }

  @Override
  public void periodic() {
    swerve.updateInputs(inputs);
    advLogger.processInputs("Swerve", inputs);
    // Update swerve module states every robot loop
    swerve.periodic();

    // Log Outputs FIXME
    advLogger.recordOutput(
        "Swerve/OdometryRotation2d(deg)", swerve.getPoseMeters().getRotation().getDegrees());
    advLogger.recordOutput("Swerve/OdometryX", swerve.getPoseMeters().getX());
    advLogger.recordOutput("Swerve/OdometryY", swerve.getPoseMeters().getY());
    advLogger.recordOutput("Swerve/Odometry", swerve.getPoseMeters());
    advLogger.recordOutput("Swerve/GyroRotation2d(deg)", swerve.getGyroRotation2d().getDegrees());

    switch (currDriveState) {
      case IDLE:
        break;
      case AUTO_BALANCE_EDGE:
        // if the gyro measures past the edge threshold, go to auto balance drive state
        // and start a
        // timer and keep driving
        if (Math.abs(swerve.getGyroPitch()) - Math.abs(tempRoll)
            >= DriveConstants.kAutoBalanceEdgeTriggerThreshold) {
          autoBalanceTimer.reset();
          autoBalanceTimer.start();
          logger.info("{} -> AUTO_BALANCE_DRIVE", currDriveState);
          currDriveState = DriveStates.AUTO_BALANCE_DRIVE;
        }
        break;
      case AUTO_BALANCE_DRIVE:
        // keep driving and once the timer reaches a certain point, switch to the
        // averaging state
        // and restart the timer and make a variable for the gyro direction
        if (autoBalanceTimer.hasElapsed(DriveConstants.kAutoBalanceSlowdownTimeSec)) {
          logger.info("{} -> AUTO_BALANCE_AVERAGE", currDriveState);
          currDriveState = DriveStates.AUTO_BALANCE_AVERAGE;
          autoBalanceTimer.restart();
          autoBalanceGyroDirection = swerve.getGyroPitch() - tempRoll;
        }
        break;
      case AUTO_BALANCE_AVERAGE:
        // every time this case is run, an average gyro pitch is collected.
        sumRoll += swerve.getGyroPitch();
        autoBalanceAvgCount++;
        logger.info("AutoBalanceAverageCount: {}", autoBalanceAvgCount);

        // after the timer reaches a certain value, the robot slows down and the timer
        // resets
        if (autoBalanceTimer.hasElapsed(DriveConstants.kAutoBalanceLoopFixTimer)) {
          avgStartingRoll = sumRoll / autoBalanceAvgCount;
          logger.info("Average Starting Roll: {}", avgStartingRoll);
          logger.info("{} -> AUTO_BALANCE", currDriveState);
          currDriveState = DriveStates.AUTO_BALANCE;
          if (isOnAllianceSide) move(DriveConstants.kAutoBalanceSlowDriveVel, 0, 0, false);
          else move(-DriveConstants.kAutoBalanceSlowDriveVel, 0, 0, false);
          autoBalanceTimer.reset();
        }
        break;
      case AUTO_BALANCE:
        // once the robot starts to balance, we stop the robot and xLock
        if (Math.abs(avgStartingRoll) - Math.abs(swerve.getGyroPitch())
            >= DriveConstants.kAutoBalanceStopThresholdDegrees) {
          drive(0, 0, 0);
          xLock();
          logger.info(
              "AutoBalance Stop: Gyro Roll: {}, trigger Difference: {}",
              swerve.getGyroPitch(),
              Math.abs(avgStartingRoll) - Math.abs(swerve.getGyroPitch()));

          // the robot goes into recovery mode which has a seperate timer
          logger.info("{} -> AUTO_BALANCE_RECOVERY", currDriveState);
          currDriveState = DriveStates.AUTO_BALANCE_RECOVERY;

          autoBalanceRecoveryTimer.reset();
          autoBalanceRecoveryTimer.start();
          recoveryStartingPitchSet = false;
        }
        break;
      case AUTO_BALANCE_RECOVERY:
        // the recovery phase in the case that the robot isn't stable on the charging
        // station
        if (autoBalanceRecoveryTimer.hasElapsed(DriveConstants.kSettleTime)) {
          autoBalanceRecoveryTimer.stop();
          // isOnAllianceSide is true move positive
          if (!recoveryStartingPitchSet) {
            recoveryStartingPitch = swerve.getGyroPitch();
            recoveryStartingPitchSet = true;
          }

          if ((Math.abs(tempRoll - swerve.getGyroPitch()) > 2)
              && (Math.abs(recoveryStartingPitch) - Math.abs(swerve.getGyroPitch()) < 1.5)) {

            if ((autoBalanceGyroDirection > 0)) {
              // Positive Gyro Direction
              if ((swerve.getGyroPitch() - tempRoll) > 0) {
                if (isOnAllianceSide)
                  move(DriveConstants.kAutoBalanceRecoveryDriveVel, 0, 0, false);
                else move(-DriveConstants.kAutoBalanceRecoveryDriveVel, 0, 0, false);
              } else if ((swerve.getGyroPitch() - tempRoll) < 0) {
                if (isOnAllianceSide)
                  move(-DriveConstants.kAutoBalanceRecoveryDriveVel, 0, 0, false);
                else move(DriveConstants.kAutoBalanceRecoveryDriveVel, 0, 0, false);
              }
            } else if ((autoBalanceGyroDirection < 0)) {
              // negative gyro direction
              if ((swerve.getGyroPitch() - tempRoll) > 0) {
                if (isOnAllianceSide)
                  move(-DriveConstants.kAutoBalanceRecoveryDriveVel, 0, 0, false);
                else move(DriveConstants.kAutoBalanceRecoveryDriveVel, 0, 0, false);
              } else if ((swerve.getGyroPitch() - tempRoll) < 0) {
                if (isOnAllianceSide)
                  move(DriveConstants.kAutoBalanceRecoveryDriveVel, 0, 0, false);
                else move(-DriveConstants.kAutoBalanceRecoveryDriveVel, 0, 0, false);
              }
            } // 9 if pos or like -8 if neg
          } else {
            drive(0, 0, 0);
            xLock();
            logger.info("{} -> AUTO_BALANCE_FINISHED", currDriveState);
            currDriveState = DriveStates.AUTO_BALANCE_FINISHED;
          }
        }
        break;

      case AUTO_BALANCE_FINISHED:
        // the robot is balanced
        autoBalanceAvgCount = 0;
        sumRoll = 0;
        avgStartingRoll = 0;
        break;

      case AUTO_BALANCE_PULSE:
        if (!autoBalanceTempStable
            && autoBalancePulseTimer.hasElapsed(DriveConstants.kPulseAutoBalanceTime)) {
          if ((!isOnAllianceSide && robotStateSubsystem.getAllianceColor() == Alliance.Blue)
              || (robotStateSubsystem.getAllianceColor() == Alliance.Red && isOnAllianceSide))
            move(DriveConstants.kHoldSpeed, 0, 0, false);
          else move(-DriveConstants.kHoldSpeed, 0, 0, false);
          logger.info("{} -> AUTO_BALANCE_CHECK", currDriveState);
          currDriveState = DriveStates.AUTO_BALANCE_CHECK;
          autoBalancePulseTimer.reset();
        }
        break;
      case AUTO_BALANCE_CHECK:
        if (isWithinThresholdAutoBalance(DriveConstants.kAutoBalanceCloseEnoughDeg)) {
          autoBalanceStableCounts++;
          autoBalanceTempStable = true;
        } else {
          autoBalanceStableCounts = 0;
        }
        if (autoBalancePulseTimer.hasElapsed(DriveConstants.kPauseAutoBalanceTime)
            && !(autoBalanceStableCounts >= DriveConstants.kAutoBalanceStableCount)) {
          double tempSpeed = DriveConstants.kPulseSpeed;
          if (Math.abs(swerve.getGyroPitch()) - 2.9 <= 10) {
            logger.info("AutoBalance Slowing to temp speed");
            tempSpeed = 0.3;
          }
          logger.info("{} -> AUTO_BALANCE_PULSE", currDriveState);
          currDriveState = DriveStates.AUTO_BALANCE_PULSE;
          autoBalancePulseTimer.reset();
          if ((!isOnAllianceSide && robotStateSubsystem.getAllianceColor() == Alliance.Blue)
              || (robotStateSubsystem.getAllianceColor() == Alliance.Red && isOnAllianceSide))
            move(tempSpeed, 0, 0, false);
          else move(-tempSpeed, 0, 0, false);
          break;
        }
        if ((autoBalanceStableCounts >= DriveConstants.kAutoBalanceStableCount)) {
          logger.info("{} -> AUTO_BALANCE_XLOCK", currDriveState);
          currDriveState = DriveStates.AUTO_BALANCE_XLOCK;
          drive(0, 0, 0);
          xLock();
        }
        break;
      case AUTO_BALANCE_XLOCK:
        break;
      default:
        break;
    }
  }

  public enum DriveStates {
    IDLE,
    AUTO_BALANCE_EDGE,
    AUTO_BALANCE_DRIVE,
    AUTO_BALANCE_AVERAGE,
    AUTO_BALANCE,
    AUTO_BALANCE_RECOVERY,
    AUTO_BALANCE_FINISHED,
    AUTO_BALANCE_PULSE,
    AUTO_BALANCE_CHECK,
    AUTO_BALANCE_XLOCK;
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    // TODO Auto-generated method stub
    super.registerWith(telemetryService);
    swerve.registerWith(telemetryService);
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("Gyro Rotation2D(deg)", () -> swerve.getGyroRotation2d().getDegrees()),
        new Measure("Odometry X", () -> swerve.getPoseMeters().getX()),
        new Measure("Odometry Y", () -> swerve.getPoseMeters().getY()),
        new Measure(
            "Odometry Rotation2D(deg)", () -> swerve.getPoseMeters().getRotation().getDegrees()),
        new Measure("Trajectory Vel", () -> holoContInput.velocityMetersPerSecond),
        new Measure("Trajectory Accel", () -> holoContInput.accelerationMetersPerSecondSq),
        new Measure("Trajectory X", () -> holoContInput.poseMeters.getX()),
        new Measure("Trajectory Y", () -> holoContInput.poseMeters.getY()),
        new Measure(
            "Trajectory Rotation2D(deg)",
            () -> holoContInput.poseMeters.getRotation().getDegrees()),
        new Measure("Desired Gyro Heading(deg)", () -> holoContAngle.getDegrees()),
        new Measure("Holonomic Cont Vx", () -> holoContOutput.vxMetersPerSecond),
        new Measure("Holonomic Cont Vy", () -> holoContOutput.vyMetersPerSecond),
        new Measure("Holonomic Cont Vomega", () -> holoContOutput.omegaRadiansPerSecond),
        new Measure("Trajectory Active", () -> trajectoryActive),
        new Measure("Timestamp X", () -> timestampedPose.getPose().getX()),
        new Measure("Timestamp Y", () -> timestampedPose.getPose().getY()),
        new Measure("Wheel 0 Angle", () -> swerve.getSwerveModuleStates()[0].angle.getDegrees()),
        new Measure("Wheel 0 Speed", () -> swerve.getSwerveModuleStates()[0].speedMetersPerSecond),
        new Measure("Wheel 1 Angle", () -> swerve.getSwerveModuleStates()[1].angle.getDegrees()),
        new Measure("Wheel 1 Speed", () -> swerve.getSwerveModuleStates()[1].speedMetersPerSecond),
        new Measure("Wheel 2 Angle", () -> swerve.getSwerveModuleStates()[2].angle.getDegrees()),
        new Measure("Wheel 2 Speed", () -> swerve.getSwerveModuleStates()[2].speedMetersPerSecond),
        new Measure("Wheel 3 Angle", () -> swerve.getSwerveModuleStates()[3].angle.getDegrees()),
        new Measure("Wheel 3 Speed", () -> swerve.getSwerveModuleStates()[3].speedMetersPerSecond),
        new Measure("FWD Vel", () -> lastVelocity[0]),
        new Measure("STR Vel", () -> lastVelocity[1]),
        new Measure("YAW Vel", () -> lastVelocity[2]));
  }
}
