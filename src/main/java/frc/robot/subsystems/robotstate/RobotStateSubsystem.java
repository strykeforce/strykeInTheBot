package frc.robot.subsystems.robotstate;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.Set;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class RobotStateSubsystem extends MeasurableSubsystem {
  private DriveSubsystem driveSubsystem;

  public RobotStateSubsystem(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
  }

  private Alliance allianceColor = DriverStation.getAlliance();

  public void setAllianceColor(Alliance alliance) {
    // logger.info("Changing to {}", alliance);
    this.allianceColor = alliance;
  }

  public Alliance getAllianceColor() {
    return allianceColor;
  }

  public void toAutoBalance(boolean isOnAllianceSide) {
    // logger.info("{} -> AUTO_BALANCE", currRobotState);
    // currRobotState = RobotState.AUTO_BALANCE;
    driveSubsystem.autoBalance(isOnAllianceSide);
  }

  public void toPulseAutoBalance(boolean isOnAllianceSide) {
    // driveSubsystem.pulseAutoBalance(isOnAllianceSide);
    // logger.info("{} -> AUTO_BALANCE", currRobotState);
    // currRobotState = RobotState.AUTO_BALANCE;
  }

  @Override
  public Set<Measure> getMeasures() {
    // TODO Auto-generated method stub
    return null;
  }
}
