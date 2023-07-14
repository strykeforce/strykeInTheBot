package frc.robot.subsystems.Arm;

import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.Extendo.ExtendoSubsystem;
import frc.robot.subsystems.RobotState.RobotStateSubsystem.gamePiece;
import frc.robot.subsystems.Wrist.WristSubsystem;
import frc.robot.subsystems.shoulder.ShoulderSubsystem;
import java.util.Set;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ArmSubsystem extends MeasurableSubsystem {

  private final ShoulderSubsystem shoulderSubsystem;
  private final ExtendoSubsystem extendoSubsystem;
  private final WristSubsystem wristSubsystem;
  private ArmState currState;
  private ArmState desiredState;

  public ArmSubsystem(
      ShoulderSubsystem shoulderSubsystem,
      ExtendoSubsystem extendoSubsystem,
      WristSubsystem wristSubsystem) {
    this.shoulderSubsystem = shoulderSubsystem;
    this.extendoSubsystem = extendoSubsystem;
    this.wristSubsystem = wristSubsystem;
    currState = ArmState.STOW;
    desiredState = ArmState.STOW;
  }

  public void stow() {
    shoulderSubsystem.setPos(ArmConstants.kStowShoulderPos);
    extendoSubsystem.setPos(ArmConstants.kStowExtendoPos);
    wristSubsystem.setPos(ArmConstants.kStowWristPos);
    currState = ArmState.PARALLEL_TRANS;
    desiredState = ArmState.STOW;
  }

  public void levelThree(gamePiece piece) {}

  public boolean isFinished() {
    if (shoulderSubsystem.isFinished()
        && extendoSubsystem.isFinished()
        && wristSubsystem.isFinished()) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    if (this.isFinished()) {
      currState = desiredState;
    }
  }

  public enum ArmState {
    SHELF_CONE,
    SHELF_CUBE,
    FLOOR_CONE,
    FLOOR_CONE_UPRIGHT,
    FLOOR_CUBE,
    HIGH_CUBE,
    HIGH_CONE,
    MID_CUBE,
    MID_CONE,
    LOW_CUBE,
    LOW_CONE,
    STOW,
    PARALLEL_TRANS,
    STOW_TO_LVL3
  }

  @Override
  public Set<Measure> getMeasures() {
    // TODO Fill in measures for grapher
    return null;
  }
}
