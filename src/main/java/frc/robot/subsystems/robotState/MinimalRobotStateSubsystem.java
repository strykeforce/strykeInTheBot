package frc.robot.subsystems.robotState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.hand.HandSubsystem;
import frc.robot.subsystems.hand.HandSubsystem.HandStates;
import frc.robot.subsystems.shoulder.MinimalShoulderSubsystem;
import frc.robot.subsystems.shoulder.ShoulderSubsystem;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class MinimalRobotStateSubsystem extends SubsystemBase {
    private DriveSubsystem driveSubsystem;
    private MinimalShoulderSubsystem shoulderSubsystem;
    private HandSubsystem handSubsystem;
    private Alliance allianceColor = DriverStation.getAlliance();
    private Logger logger = LoggerFactory.getLogger(RobotStateSubsystem.class);
    private RobotState currRobotState = RobotState.STOW;
    private RobotState nextRobotState = RobotState.STOW;
    private GamePiece currentPiece = GamePiece.NONE;
    private GamePiece targetPiece = GamePiece.NONE;
    private TargetLevel targetLevel = TargetLevel.NONE;
    private TargetCol targetCol = TargetCol.NONE;
    private double currShelfPoseX;
    private boolean isConePickupUpright = true;

    public MinimalRobotStateSubsystem(DriveSubsystem driveSubsystem, ShoulderSubsystem shoulderSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.shoulderSubsystem = shoulderSubsystem;
    }

    public Alliance getAllianceColor() {
        return allianceColor;
    }

    public void setAllianceColor(Alliance alliance) {
        // logger.info("Changing to {}", alliance);
        this.allianceColor = alliance;
    }

    public RobotState getCurrRobotState() {
        return currRobotState;
    }

    private void setRobotStateLogged(RobotState robotState) {
        if (this.currRobotState != robotState) {
            logger.info("{} -> {}", this.currRobotState, robotState);
            this.currRobotState = robotState;
        }
    }

    public GamePiece getCurrentPiece() {
        return currentPiece;
    }

    public TargetLevel getTargetLevel() {
        return targetLevel;
    }

    public void setTargetLevel(TargetLevel targetLevel) {
        this.targetLevel = targetLevel;
    }

    public TargetCol getTargetCol() {
        return targetCol;
    }

    public void setTargetCol(TargetCol targetCol) {
        this.targetCol = targetCol;
    }

    public GamePiece getTargetPiece() {
        return targetPiece;
    }

    public void setTargetPiece(GamePiece targetPiece) {
        this.targetPiece = targetPiece;
    }

    public boolean isStowed() {
        return currRobotState == RobotState.STOW;
    }

    public boolean isShelfPieceCube() {
        // FIXME maybe flip isBlueAlliance boolean
        double angle = driveSubsystem.getGyroRotation2d().getRadians();
        if (isBlueAlliance()) {
            return (angle > Math.PI * -0.5) && (angle < Math.PI * 0.5);
        } else {
            return (angle < Math.PI * -0.5) || (angle > Math.PI * 0.5);
        }
    }

    public boolean isBlueAlliance() {
        return allianceColor == Alliance.Blue;
    }

    public void toStow(RobotState nextState) {
        nextRobotState = nextState;
        handSubsystem.idle();
        shoulderSubsystem.stow();
        setRobotStateLogged(RobotState.TO_STOW);
    }

    public void toStow() {
        toStow(RobotState.STOW);
    }

    public void toManualScore() {
        logger.info("starting manual score");

        if (isStowed()) {
            setRobotStateLogged(RobotState.TO_MANUAL_SCORE);
        } else {
            toStow(RobotState.MANUAL_SCORE);
        }
    }
    
    public void toManualSubstation() {
        logger.info("starting manual shelf");
    
        if (isStowed()) {
          if (isShelfPieceCube()) {
            shoulderSubsystem.shelf(GamePiece.CUBE);
          } else {
            shoulderSubsystem.shelf(GamePiece.CONE);
          }
          setRobotStateLogged(RobotState.TO_MANUAL_SUBSTATION);
        } else {
          toStow(RobotState.MANUAL_SUBSTATION);
        }
      }

    public void toReleaseGamepiece() {
        logger.info("starting release gamepiece");

        setRobotStateLogged(RobotState.RELEASE_GAMEPIECE);
    }

    public void toAutobalance() {
        logger.info("starting autobalance");

        if (isStowed()) {
            // TODO: add autobalance functionality
            setRobotStateLogged(RobotState.TO_AUTOBALANCE);
        } else {
            toStow(RobotState.AUTOBALANCE);
        }
    }

    @Override
    public void periodic() {
        switch (currRobotState) {
            case STOW:
                switch (nextRobotState) {
                    default:
                        break;
                }
                break;

            case MANUAL_SCORE:
                break;
            case RELEASE_GAMEPIECE:
                if (handSubsystem.getState() == HandStates.IDLE) {
                    currentPiece = GamePiece.NONE;
                    toStow();
                } else
                    handSubsystem.ejectPiece();
                break;
            case TO_STOW:
                if (!shoulderSubsystem.isFinished())
                    break;
                setRobotStateLogged(RobotState.STOW);
                break;
            case TO_MANUAL_SUBSTATION:
                if (!shoulderSubsystem.isFinished())
                    break;
                handSubsystem.grabPiece();
                setRobotStateLogged(RobotState.MANUAL_SUBSTATION);
                break;
            case TO_MANUAL_SCORE:
                if (!shoulderSubsystem.isFinished())
                    break;
                setRobotStateLogged(RobotState.MANUAL_SCORE);
                break;
            case MANUAL_SUBSTATION:
                if (!handSubsystem.hasGamePiece())
                    break;
                currentPiece = targetPiece;
                currShelfPoseX = driveSubsystem.getPoseMeters().getX();
                setRobotStateLogged(RobotState.SHELF_WAIT);
                break;
            case SHELF_WAIT:
                double desiredPoseX;
                // TODO: Logic may differ than what is present below
                if (isBlueAlliance()) {
                    desiredPoseX = currShelfPoseX - MinimalShoulderSubsystem.kShelfMove;
                    if (driveSubsystem.getPoseMeters().getX() <= desiredPoseX) {
                        toStow();
                    }
                } else {
                    desiredPoseX = currShelfPoseX + MinimalShoulderSubsystem.kShelfMove;
                    if (driveSubsystem.getPoseMeters().getX() >= desiredPoseX) {
                        toStow();
                    }
                }
                break;
            case TO_AUTOBALANCE:
                // TODO: add autobalance functionality
                break;
            default:
                logger.warn("{} is an invalid robot state!", currRobotState);
                break;
        }
    }

    public enum RobotState {
        STOW,
        FLOOR_PICKUP,
        AUTO_SHELF,
        MANUAL_SUBSTATION,
        SHELF_WAIT,
        AUTO_SCORE,
        MANUAL_SCORE,
        RELEASE_GAMEPIECE,
        AUTOBALANCE,
        TO_STOW,
        TO_FLOOR_PICKUP,
        TO_AUTO_SHELF,
        TO_MANUAL_SUBSTATION,
        TO_AUTO_SCORE,
        TO_MANUAL_SCORE,
        TO_AUTOBALANCE,
    }

    public enum GamePiece {
        NONE,
        CUBE,
        CONE,
    }

    public enum TargetLevel {
        NONE,
        LOW,
        MID,
        HIGH,
    }

    public enum TargetCol {
        NONE,
        LEFT,
        MID,
        RIGHT,
    }
}
