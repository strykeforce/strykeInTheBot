package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.timestampedpose.TimestampedPose;
import java.io.IOException;
import java.util.List;
import java.util.Set;
import net.jafama.FastMath;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class VisionSubsystem extends MeasurableSubsystem {
  private static final Logger logger = LoggerFactory.getLogger(VisionSubsystem.class);
  private DriveSubsystem driveSubsystem;
  private PhotonCamera camera = new PhotonCamera("idkfixthisprobably");
  private PhotonPoseEstimator photonPoseEstimator;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private Translation2d robotPose;
  private PhotonPipelineResult result;
  private List<PhotonTrackedTarget> targets;
  private PhotonTrackedTarget bestTarget;
  private CircularBuffer gyroBuffer = new CircularBuffer(10000);
  private CircularBuffer timestampBuffer = new CircularBuffer(10000);
  private CircularBuffer velocityBuffer = new CircularBuffer(10000);
  private double timeStamp;
  private int gyroBufferId = 0;
  private boolean buffersFull = false;
  private boolean canFillBuffers = false;
  private EstimatedRobotPose savedOffRobotEstimation;
  private Pose3d cameraPose;
  private Pose3d lastPose;
  private int currUpdate = 0;
  private int lastUpdate = 0;
  private int visionOff = 0;
  private final Notifier photonVisionThread;

  public VisionSubsystem(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.photonVisionThread = new Notifier(this::visionUpdateThread);
    photonVisionThread.startPeriodic(20.0 / 1000.0);

    try {
      aprilTagFieldLayout =
          new AprilTagFieldLayout(
              Filesystem.getDeployDirectory().toPath() + "/2023-chargedup.json");
    } catch (IOException e) {
      logger.error("VISION SUBSYSTEM : APRILTAG JSON FAILED");
    }

    photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.LOWEST_AMBIGUITY,
            camera,
            new Transform3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(0))));
  }

  public Translation2d getOdometry() {
    return robotPose;
  }

  public TimestampedPose odomNewPoseViaVision() {
    // List<PhotonTrackedTarget> gyroBuffer;
    if (buffersFull) {
      Rotation2d gyroAngle =
          new Rotation2d(
              gyroBuffer.get(
                  VisionConstants.kBufferLookupOffset)); // driveSubsystem.getGyroRotation2d();
      // List<PhotonTrackedTarget> timestampBuffer;
      double timestamp = timestampBuffer.get(VisionConstants.kBufferLookupOffset);

      Pose2d odomPose = new Pose2d(robotPose, gyroAngle);
      return new TimestampedPose((long) timestamp, odomPose);
    } else {
      return new TimestampedPose((long) 2767, new Pose2d());
    }
  }

  public void setFillBuffers(boolean set) {
    logger.info("Set Fill Buffers: {}", set);
    canFillBuffers = set;
  }

  public void fillBuffers() {
    gyroBuffer.addFirst(driveSubsystem.getGyroRotation2d().getRadians());
    timestampBuffer.addFirst(RobotController.getFPGATime());
    velocityBuffer.addFirst(driveSubsystem.getVectorSpeed());
    buffersFull = true;
  }

  public double getNumTargets() {
    return targets.size();
  }

  public Translation2d getCameraOffset() {
    return new Translation2d(
        VisionConstants.kCameraOffset
            * FastMath.cos(
                Units.degreesToRadians(
                    VisionConstants.kCameraAngleOffset
                        - driveSubsystem.getGyroRotation2d().getDegrees())),
        -VisionConstants.kCameraOffset
            * FastMath.sin(
                Units.degreesToRadians(
                    VisionConstants.kCameraAngleOffset
                        - driveSubsystem.getGyroRotation2d().getDegrees())));
  }

  public Translation2d getPositionFromRobot() {
    if (result.hasTargets()) {
      double x =
          bestTarget.getBestCameraToTarget().getX(); // + Constants.VisionConstants.kXCameraOffset;
      double y =
          bestTarget.getBestCameraToTarget().getY(); // + Constants.VisionConstants.kYCameraOffset;
      return new Translation2d(x, y); // z is from the camera height
    }
    return new Translation2d(2767, 2767);
  }

  public boolean getHasTargets() {
    return result.hasTargets();
  }

  @Override
  public void periodic() {
    if (driveSubsystem.canGetVisionUpdates() && lastUpdate < currUpdate) {
      lastUpdate = currUpdate;
      double y = cameraPose.getY();
      double x = cameraPose.getX();

      if (FastMath.hypot(
                  x - driveSubsystem.getPoseMeters().getX(),
                  y - driveSubsystem.getPoseMeters().getY())
              <= 0.75
          || (visionOff > 0 && FastMath.hypot(x - lastPose.getX(), y - lastPose.getY()) <= 0.75)) {
        visionOff = 0;
        driveSubsystem.updateOdometryWithVision(
            new Pose2d(
                new Translation2d(x, y).plus(getCameraOffset()),
                new Rotation2d(gyroBuffer.get(gyroBufferId))),
            (long) timeStamp);

        // if (driveSubsystem.canGetVisionUpdates() && driveSubsystem.isAutoDriving()) {
        //   driveSubsystem.resetOdometryNoLog(
        //       new Pose2d(
        //           new Translation2d(x, y).plus(getCameraOffset()),
        //           new Rotation2d(gyroBuffer.get(gyroBufferId))));
        // }
      } else {
        logger.info("bad reading");
        visionOff++;
        lastPose = cameraPose;
      }
      robotPose = new Translation2d(x, y);
    }
  }

  private void visionUpdateThread() {
    try {
      result = camera.getLatestResult();
      if (canFillBuffers) fillBuffers();
    } catch (Exception e) {
    }

    if (result.hasTargets()) {
      targets = result.getTargets();
      bestTarget = result.getBestTarget();
      timeStamp = result.getTimestampSeconds();
      gyroBufferId = (int) FastMath.floor(result.getLatencyMillis() / 20);

      if (result.getBestTarget().getPoseAmbiguity() <= 0.15 || result.targets.size() > 1) {
        try {
          savedOffRobotEstimation = photonPoseEstimator.update().get();
          cameraPose = savedOffRobotEstimation.estimatedPose;
          currUpdate++;
        } catch (Exception e) {
        }
      }
    }
  }

  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        new Measure("Has Targets", () -> getHasTargets() ? 1.0 : 0.0),
        new Measure("Camera Offset X", () -> getCameraOffset().getX()),
        new Measure("Camera Offset Y", () -> getCameraOffset().getY()));
  }
}
