package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.drive.DriveSubsystem;
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
  private static DriveSubsystem driveSubsystem;
  private PhotonCamera camera = new PhotonCamera("idkfixthisprobably");
  private PhotonPoseEstimator photonPoseEstimator;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private Translation2d robotPose;
  private PhotonPipelineResult result;
  private List<PhotonTrackedTarget> targets;
  private PhotonTrackedTarget bestTarget;
  private double timeStamp;
  private int gyroBufferId = 0;
  private boolean canFillBuffers = false;
  private EstimatedRobotPose savedOffRobotEstimation;
  private Pose3d cameraPose;
  private Pose3d lastPose;
  private int currUpdate = 0;
  private int lastUpdate = 0;

  public VisionSubsystem(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

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

  public void fillBuffers() {}

  @Override
  public void periodic() {
    if (driveSubsystem.canGetVisionUpdates() && lastUpdate < currUpdate) {
      lastUpdate = currUpdate;
    }
  }

  private void visionUpdateThread() {
    try {
      result = camera.getLatestResult();
      if (canFillBuffers) fillBuffers();
    } catch (Exception e) {}

    if (result.hasTargets()) {
      targets = result.getTargets();
      bestTarget = result.getBestTarget();
      timeStamp = result.getTimestampSeconds();
      gyroBufferId = (int) FastMath.floor(result.getLatencyMillis() / 20);

      if (result.hasTargets()
          && (result.getBestTarget().getPoseAmbiguity() <= 0.15 || result.targets.size() > 1)) {
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
    // TODO Auto-generated method stub
    return null;
  }
}
