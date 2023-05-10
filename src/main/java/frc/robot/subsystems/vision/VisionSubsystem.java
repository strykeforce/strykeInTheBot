package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class VisionSubsystem extends MeasurableSubsystem {
  private PhotonCamera cam = new PhotonCamera("idkfixthisprobably");
  private static final Logger logger = LoggerFactory.getLogger(VisionSubsystem.class);
  private static DriveSubsystem driveSubsystem;
  private Pose3d lastPose;
  private Pose3d cameraPose = new Pose3d(new Translation3d(2767.0, 2767.0, 0.0), new Rotation3d());
  private final Notifier photonVisionThread;
  private PhotonPipelineResult result;
  private List<PhotonTrackedTarget> targets;
  private PhotonTrackedTarget bestTarget;
  private double timeStamp;
  private int gyroBufferId = 0;
  private CircularBuffer gyroBuffer = new CircularBuffer(10000);
  private CircularBuffer timestampBuffer = new CircularBuffer(10000);
  private CircularBuffer velocityBuffer = new CircularBuffer(10000);
  private boolean canFillBuffers = false;
  private boolean buffersFull = false;

  public VisionSubsystem(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    photonVisionThread = new Notifier(this::visionUpdateThread);
    photonVisionThread.startPeriodic(20.0 / 1000.0);
  }

  public void fillBuffers() {
    // gyroBuffer.addFirst(driveSubsystem.getGyroRotation2d().getRadians());
    timestampBuffer.addFirst(RobotController.getFPGATime());
    // velocityBuffer.addFirst(driveSubsystem.getVectorSpeed());
    buffersFull = true;
  }

  private void visionUpdateThread() {}

  @Override
  public Set<Measure> getMeasures() {
    // TODO Auto-generated method stub
    return null;
  }
}
