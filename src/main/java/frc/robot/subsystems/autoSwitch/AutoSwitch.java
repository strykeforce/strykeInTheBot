package frc.robot.subsystems.autoSwitch;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.commands.auto.AutoCommandInterface;
import frc.robot.commands.auto.DefaultAutoCommand;
// import frc.robot.commands.auto.DefaultAutoCommand;
// import frc.robot.commands.auto.DoNothingAutonCommand;
import frc.robot.commands.auto.newAuto4Piece;
import frc.robot.constants.AutonConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.hand.HandSubsystem;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem;
import frc.robot.subsystems.shoulder.MinimalShoulderSubsystem;
import java.util.ArrayList;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.thirdcoast.util.AutonSwitch;

public class AutoSwitch {
  private final AutonSwitch autoSwitch;
  private ArrayList<DigitalInput> switchInputs = new ArrayList<>();
  private int currAutoSwitchPos = -1;
  private int newAutoSwitchPos;
  private int autoSwitchStableCounts = 0;
  private Logger logger = LoggerFactory.getLogger(AutoSwitch.class);

  private AutoCommandInterface autoCommand;
  private final MinimalRobotStateSubsystem robotStateSubsystem;
  private final DriveSubsystem driveSubsystem;
  // private final ArmSubsystem armSubsystem;
  private final MinimalShoulderSubsystem shoulderSubsystem;
  private final HandSubsystem handSubsystem;
  //   private final VisionSubsystem visionSubsystem;
  AutoCommandInterface defaultCommand;

  public AutoSwitch(
      MinimalRobotStateSubsystem robotStateSubsystem,
      DriveSubsystem driveSubsystem,
      HandSubsystem handSubsystem,
      MinimalShoulderSubsystem shoulderSubsystem) {
    // VisionSubsystem visionSubsystem
    this.robotStateSubsystem = robotStateSubsystem;
    this.driveSubsystem = driveSubsystem;
    // this.armSubsystem = armSubsystem;
    this.shoulderSubsystem = shoulderSubsystem;
    this.handSubsystem = handSubsystem;

    // defaultCommand =
    //     new MiddleToDockWithMobility(
    //         driveSubsystem,
    //         robotStateSubsystem,
    //         shoulderSubsystem,
    //         handSubsystem,
    //         "piecePlaceOverLinePath",
    //         "tinyLittleToBalancePath");

    defaultCommand =
        new DefaultAutoCommand(
            driveSubsystem, robotStateSubsystem, handSubsystem, shoulderSubsystem);

    // new DefaultAutoCommand(
    //     driveSubsystem, robotStateSubsystem, handSubsystem, shoulderSubsystem);

    for (int i = AutonConstants.kStartSwitchID; i <= AutonConstants.kEndSwitchId; i++) {
      switchInputs.add(new DigitalInput(i));
    }
    autoSwitch = new AutonSwitch(switchInputs);
  }

  public void checkSwitch() {
    if (hasSwitchChanged()) {
      logger.info(
          "Initializing Auto Switch Position: {}", String.format("%02X", currAutoSwitchPos));
      autoCommand = getAutoCommand(currAutoSwitchPos);
      if (!autoCommand.hasGenerated()) autoCommand.generateTrajectory();
    }
  }

  public void resetSwitchPos() {
    if (currAutoSwitchPos == -1) {
      logger.info("Reset Auto Switch");
    }
    currAutoSwitchPos = -1;
  }

  public AutoCommandInterface getAutoCommand() {
    if (autoCommand == null) {
      return defaultCommand;
    } else return this.autoCommand;
  }

  private boolean hasSwitchChanged() {
    boolean changed = false;
    int switchPos = autoSwitch.position();

    if (switchPos != newAutoSwitchPos) {
      autoSwitchStableCounts = 0;
      newAutoSwitchPos = switchPos;
    } else autoSwitchStableCounts++;

    if (autoSwitchStableCounts > AutonConstants.kSwitchStableCounts
        && currAutoSwitchPos != newAutoSwitchPos) {
      changed = true;
      currAutoSwitchPos = newAutoSwitchPos;
    }

    return changed;
  }

  private AutoCommandInterface getAutoCommand(int switchPos) {
    switch (switchPos) {
        // Non-Bump Side
        // case 0x00:
        //   // Cube lvl 2, mobility
        //   return new PiecePlaceMobility(
        //       driveSubsystem, robotStateSubsystem, shoulderSubsystem, handSubsystem,
        // "fetchPath");

        // case 0x10:
        //   return new MiddleToDockWithMobility(
        //       driveSubsystem,
        //       robotStateSubsystem,
        //       shoulderSubsystem,
        //       handSubsystem,
        //       "piecePlaceOverLinePath",
        //       "tinyLittleToBalancePath");
        //   // Bump Side
        // case 0x20:
        //   // Cube lvl 2, mobility, bump side
        //   return new PiecePlaceMobilityBump(
        //       driveSubsystem, robotStateSubsystem, shoulderSubsystem, handSubsystem,
        // "fetchBumpPath");
        // case 0x34:
        //   // 2024 test auton
        //   return new newAuto4Piece(driveSubsystem, robotStateSubsystem, "fiveMeterPath");
      default:
        // 2024 test auton
        return new newAuto4Piece(driveSubsystem, robotStateSubsystem, "fiveMeterPath");
        // String msg = String.format("no auto command assigned for switch pos: %02X", switchPos);
        // DriverStation.reportWarning(msg, false);
        // return new DefaultAutoCommand(
        //     driveSubsystem, robotStateSubsystem, handSubsystem, shoulderSubsystem);
    }
  }
}
