package frc.robot.subsystems.autoSwitch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.AutonConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.hand.HandSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.shoulder.ShoulderSubsystem;
import frc.robot.commands.auto.AutoCommandInterface;
import frc.robot.commands.auto.DefaultAutoCommand;
// import frc.robot.commands.auto.DefaultAutoCommand;
// import frc.robot.commands.auto.DoNothingAutonCommand;
import frc.robot.commands.auto.MiddleToDockWithMobility;
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
  private final RobotStateSubsystem robotStateSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final ArmSubsystem armSubsystem;
  private final HandSubsystem handSubsystem;
//   private final VisionSubsystem visionSubsystem;
  AutoCommandInterface defaultCommand;

  public AutoSwitch(
      RobotStateSubsystem robotStateSubsystem,
      DriveSubsystem driveSubsystem,
      ArmSubsystem armSubsystem,
      HandSubsystem handSubsystem) {
      // VisionSubsystem visionSubsystem 
    this.robotStateSubsystem = robotStateSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.armSubsystem = armSubsystem;
    this.handSubsystem = handSubsystem;

    defaultCommand =
        new DefaultAutoCommand(
            driveSubsystem, robotStateSubsystem, handSubsystem, armSubsystem);

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
      case 0x00:
        // // Cone Lvl 3, Cube Lvl 3, Auto Balance
        // return new TwoPieceWithDockAutoCommandGroup(
        //     driveSubsystem,
        //     robotStateSubsystem,
        //     armSubsystem,
        //     handSubsystem,
        //     "pieceOneFetchPath",
        //     "pieceOnePlacePath",
        //     "pieceTwoToDockPath");
      case 0x01:
        // // Cone Lvl 3, Cube Lvl 3
        // return new TwoPieceLvl3AutoCommandGroup(
        //     driveSubsystem,
        //     robotStateSubsystem,
        //     armSubsystem,
        //     handSubsystem,
        //     "pieceOneFetchPath",
        //     "pieceOnePlacePath");
      case 0x02:
        // // Same as 0x00 but scores cone mid
        // return new TwoPieceWithDockAutoMidCommandGroup(
        //     driveSubsystem,
        //     robotStateSubsystem,
        //     armSubsystem,
        //     handSubsystem,
        //     "pieceOneFetchPath",
        //     "pieceOnePlacePath",
        //     "pieceTwoToDockPath");
      case 0x03:
        // // Cone lvl 3, Cube lvl 3, Cube lvl 2
        // return new ThreePieceSmoothAutoCommandGroup(
        //     driveSubsystem,
        //     robotStateSubsystem,
        //     armSubsystem,
        //     handSubsystem,
        //     "pieceOneFetchCubeThreePath",
        //     "pieceOnePlaceCubeThreePath",
        //     "pieceTwoFetchCubeFourPath",
        //     "pieceTwoPlaceCubeFourPath",
        //     "pieceScoreWithoutAutoPath");
      case 0x10:
        // // Cone lvl 3, cube lvl 3, balance
        // return new TwoPieceMiddleBalanceAutoCommandGroup(
        //     driveSubsystem,
        //     robotStateSubsystem,
        //     armSubsystem,
        //     handSubsystem,
        //     "pieceFetchChargeStation",
        //     "pieceScoreChargeStation",
        //     "pieceScoreWithoutAutoChargeStation",
        //     "middleScoreToBalance");

      case 0x11:
        // return new MiddleToDock(
        //     driveSubsystem,
        //     robotStateSubsystem,
        //     armSubsystem,
        //     handSubsystem,
        //     "middleScoreToBalance");

      case 0x12:
        return new MiddleToDockWithMobility(
            driveSubsystem,
            robotStateSubsystem,
            armSubsystem,
            handSubsystem,
            "piecePlaceOverLinePath",
            "tinyLittleToBalancePath");
        // Bump Side
      case 0x20:
        // // Cone Lvl 3, Cube Lvl 3
        // return new TwoPieceLvl3BumpAutoCommandGroup(
        //     driveSubsystem,
        //     robotStateSubsystem,
        //     armSubsystem,
        //     handSubsystem,
        //     "pieceOneFetchPathBump",
        //     "pieceOneDeliverBumpPathPt1",
        //     "pieceOneDeliverBumpPathPt2");
      case 0x21:
        // // FALLBACK - Cone Lvl 3, cube lvl 2 and 3
        // return new ThreePieceBumpFallbackAutoCommandGroup(
        //     driveSubsystem,
        //     robotStateSubsystem,
        //     armSubsystem,
        //     handSubsystem,
        //     "pieceOneFetchBumpCubeOneFallback",
        //     "pieceOneDeliverBumpCubeOneFallback",
        //     "pieceTwoFetchBumpCubeTwoFallback",
        //     "pieceTwoDeliverBumpCubeTwoFallback",
        //     "pieceScoreWithoutAutoBumpFallback");
      case 0x22:
        // // Cone Lvl3, Cube Lvl3
        // return new TwoPieceBumpWithDockAutoCommandGroup(
        //     driveSubsystem,
        //     robotStateSubsystem,
        //     armSubsystem,
        //     handSubsystem,
        //     "pieceOneFetchPathBump",
        //     "pieceOneDeliverBumpPath",
        //     "pieceTwoToDockBumpPath",
        //     "pieceScoreWithoutAutoBump");
      case 0x23:
        // // Cone Lvl 3, cube lvl 2 and 3
        // return new ThreePieceBumpAutoCommandGroup(
        //     driveSubsystem,
        //     robotStateSubsystem,
        //     armSubsystem,
        //     handSubsystem,
        //     "pieceOneFetchBumpCubeTwo",
        //     "pieceOneDeliverBumpCubeTwo",
        //     "pieceTwoFetchBumpCubeOne",
        //     "pieceTwoDeliverBumpCubeOne",
        //     "pieceScoreWithoutAutoBump");
      case 0x24:
        // // Cone Lvl 1, Cube Lvl 3 and 2
        // return new ThreePieceBumpLowAutoCommandGroup(
        //     driveSubsystem,
        //     robotStateSubsystem,
        //     armSubsystem,
        //     handSubsystem,
        //     "pieceOneFetchBumpCubeTwo",
        //     "pieceOneDeliverBumpCubeTwo",
        //     "pieceTwoFetchBumpCubeOne",
        //     "pieceTwoDeliverBumpCubeOne",
        //     "pieceScoreWithoutAutoBump");
      case 0x30:
        // return new DoNothingAutonCommand(
        //     driveSubsystem,
        //     robotStateSubsystem,
        //     armSubsystem
      default:
        String msg = String.format("no auto command assigned for switch pos: %02X", switchPos);
        DriverStation.reportWarning(msg, false);
        return new DefaultAutoCommand(
            driveSubsystem, robotStateSubsystem, handSubsystem, armSubsystem);
    }
  }
}