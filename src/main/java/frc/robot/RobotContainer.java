// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.TestBalanceCommand;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.DriveTeleopCommand;
import frc.robot.commands.drive.LockZeroCommand;
import frc.robot.commands.drive.XLockCommand;
import frc.robot.commands.drive.ZeroGyroCommand;
import frc.robot.commands.robotState.ClearGamePieceCommand;
import frc.robot.commands.robotState.FloorPickupCommand;
import frc.robot.commands.robotState.ManualStageArmCommand;
import frc.robot.commands.robotState.ReleaseGamepieceCommand;
import frc.robot.commands.robotState.SetTargetLevelCommand;
import frc.robot.commands.robotState.StowCommand;
import frc.robot.commands.robotState.SubstationPickupCommand;
import frc.robot.commands.shoulder.ZeroShoulderCommand;
import frc.robot.controllers.FlyskyJoystick;
import frc.robot.controllers.FlyskyJoystick.Button;
import frc.robot.subsystems.autoSwitch.AutoSwitch;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.example.ExampleIOTalon;
import frc.robot.subsystems.example.ExampleSubsystem;
import frc.robot.subsystems.hand.HandIOFalcon;
import frc.robot.subsystems.hand.HandSubsystem;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem.GamePiece;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem.TargetLevel;
import frc.robot.subsystems.shoulder.MinimalShoulderFalconIO;
import frc.robot.subsystems.shoulder.MinimalShoulderSubsystem;
import java.util.Map;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

public class RobotContainer {
  private final Logger logger;
  // Grapher
  private final TelemetryService telemetryService = new TelemetryService(TelemetryController::new);

  // Subsystems
  private final ExampleSubsystem exampleSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final MinimalShoulderSubsystem shoulderSubsystem;
  // private final ExtendoSubsystem extendoSubsystem;
  // private final WristSubsystem wristSubsystem;
  // private final ArmSubsystem armSubsystem;
  private final MinimalRobotStateSubsystem robotStateSubsystem;
  private final HandSubsystem handSubsystem;
  private final AutoSwitch autoSwitch;

  // IO Objects
  private final Joystick driveJoystick = new Joystick(0);
  private final XboxController xboxController = new XboxController(1);

  // auton stuff
  private TestBalanceCommand balancepath;
  private DriveAutonCommand fiveMeterTest;

  private SuppliedValueWidget<Boolean> allianceColor;
  private Alliance alliance = Alliance.Invalid;
  private SuppliedValueWidget<Boolean> currGamePiece;
  private boolean isEvent = false;

  public RobotContainer() {
    logger = LoggerFactory.getLogger(RobotContainer.class);

    exampleSubsystem = new ExampleSubsystem(new ExampleIOTalon());
    shoulderSubsystem = new MinimalShoulderSubsystem(new MinimalShoulderFalconIO());
    handSubsystem = new HandSubsystem(new HandIOFalcon());
    driveSubsystem = new DriveSubsystem();
    robotStateSubsystem =
        new MinimalRobotStateSubsystem(driveSubsystem, shoulderSubsystem, handSubsystem);
    driveSubsystem.setRobotStateSubsystem(robotStateSubsystem);

    // extendoSubsystem = new ExtendoSubsystem(new ExtendoIOTalon());
    // wristSubsystem = new WristSubsystem(new WristIOTalon(), new WristEncoderIOCanandcoder());
    // armSubsystem = new ArmSubsystem(shoulderSubsystem, extendoSubsystem, wristSubsystem);

    driveSubsystem.teleResetGyro();

    autoSwitch =
        new AutoSwitch(robotStateSubsystem, driveSubsystem, handSubsystem, shoulderSubsystem);

    configureDriverButtonBindings();
    configureOperatorBindings();
    configureMatchDashboard();
    configTelemetry();
  }

  private void configureOperatorBindings() {
    // Set Level/Col
    // Level 3
    Trigger leftTrigger =
        new Trigger(() -> xboxController.getLeftTriggerAxis() >= 0.1)
            .onTrue(new SetTargetLevelCommand(robotStateSubsystem, TargetLevel.HIGH));
    Trigger rightTrigger =
        new Trigger(() -> xboxController.getRightTriggerAxis() >= 0.1)
            .onTrue(new SetTargetLevelCommand(robotStateSubsystem, TargetLevel.HIGH));

    // Level 2
    new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value)
        .onTrue(new SetTargetLevelCommand(robotStateSubsystem, TargetLevel.MID));
    new JoystickButton(xboxController, XboxController.Button.kRightBumper.value)
        .onTrue(new SetTargetLevelCommand(robotStateSubsystem, TargetLevel.MID));

    // Level 1
    Trigger floorPlace = new Trigger(() -> xboxController.getPOV() == 0);
    floorPlace.onTrue(new SetTargetLevelCommand(robotStateSubsystem, TargetLevel.LOW));

    // Shelf (substation)
    new JoystickButton(xboxController, XboxController.Button.kY.value)
        .onTrue(new SubstationPickupCommand(robotStateSubsystem, shoulderSubsystem, handSubsystem));

    // Clear gamepiece
    new JoystickButton(xboxController, XboxController.Button.kB.value)
        .onTrue(new ClearGamePieceCommand(robotStateSubsystem));

    // Floor pickup
    new JoystickButton(xboxController, XboxController.Button.kX.value)
        .onTrue(
            new FloorPickupCommand(
                robotStateSubsystem, shoulderSubsystem, handSubsystem, GamePiece.CUBE));

    // Stow
    new JoystickButton(xboxController, XboxController.Button.kBack.value)
        .onTrue(new StowCommand(robotStateSubsystem, shoulderSubsystem));
  }

  public boolean configureDriverButtonBindings() {
    String joystick = DriverStation.getJoystickName(0);
    configureFlyskyDriverButtonBindings();
    // boolean success = false;
    // switch (joystick) {
    // case "InterLink-X":
    // logger.info("Configuring Interlink Joystick");
    // configureInterlinkDriverButtonBindings();
    // success = true;
    // break;
    // case "FlySky NV14 Joystick":
    // logger.info("Configuring Flysky Joystick");
    // configureFlyskyDriverButtonBindings();
    // success = true;
    // break;
    // default:
    // logger.info("No joystick type {} defined", joystick);
    // break;
    // }
    return true;
  }

  private void configureInterlinkDriverButtonBindings() {}

  private void configureFlyskyDriverButtonBindings() {

    FlyskyJoystick flysky = new FlyskyJoystick(driveJoystick);
    driveSubsystem.setDefaultCommand(

        // DRIVE
        new DriveTeleopCommand(
            () -> flysky.getFwd(),
            () -> flysky.getStr(),
            () -> flysky.getYaw(),
            driveJoystick,
            driveSubsystem,
            robotStateSubsystem,
            handSubsystem));

    new JoystickButton(driveJoystick, Button.M_LTRIM_UP.id)
        .onTrue(new ZeroGyroCommand(driveSubsystem));

    new JoystickButton(driveJoystick, Button.M_RTRIM_UP.id)
        .onTrue(new XLockCommand(driveSubsystem));

    // new JoystickButton(driveJoystick, Button.M_RTRIM_R.id)
    //     .onTrue(new DriveAutonCommand(driveSubsystem, "fiveMeterPath", true, true));

    new JoystickButton(driveJoystick, Button.SWB_DWN.id)
        .onTrue(
            new FloorPickupCommand(
                robotStateSubsystem, shoulderSubsystem, handSubsystem, GamePiece.CUBE))
        .onFalse(
            new FloorPickupCommand(
                robotStateSubsystem, shoulderSubsystem, handSubsystem, GamePiece.CUBE));
    new JoystickButton(driveJoystick, Button.SWB_UP.id)
        .onTrue(
            new FloorPickupCommand(
                robotStateSubsystem, shoulderSubsystem, handSubsystem, GamePiece.CUBE))
        .onFalse(
            new FloorPickupCommand(
                robotStateSubsystem, shoulderSubsystem, handSubsystem, GamePiece.CUBE));

    // Zero Shoulder
    new JoystickButton(driveJoystick, Button.M_SWC.id)
        .onTrue(new ZeroShoulderCommand(shoulderSubsystem));

    // Manual stage arm
    new JoystickButton(driveJoystick, Button.M_SWE.id)
        .onTrue(new ManualStageArmCommand(robotStateSubsystem, shoulderSubsystem));

    // Release game piece
    new JoystickButton(driveJoystick, Button.M_SWH.id)
        .onTrue(new ReleaseGamepieceCommand(robotStateSubsystem, handSubsystem));

    // Stow
    new JoystickButton(driveJoystick, Button.SWD.id)
        .onTrue(new StowCommand(robotStateSubsystem, shoulderSubsystem))
        .onFalse(new StowCommand(robotStateSubsystem, shoulderSubsystem));
  }

  private void configTelemetry() {
    exampleSubsystem.registerWith(telemetryService);
    shoulderSubsystem.registerWith(telemetryService);
    handSubsystem.registerWith(telemetryService);
    robotStateSubsystem.registerWith(telemetryService);
    driveSubsystem.registerWith(telemetryService);

    telemetryService.start();
  }

  public void zeroShoulder() {
    if (!shoulderSubsystem.hasZeroed()) shoulderSubsystem.zero();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public AutoSwitch getAutoSwitch() {
    return autoSwitch;
  }

  private void configureMatchDashboard() {
    allianceColor =
        Shuffleboard.getTab("Match")
            .addBoolean("AllianceColor", () -> alliance != Alliance.Invalid)
            .withProperties(Map.of("colorWhenFalse", "black"))
            .withSize(2, 2)
            .withPosition(0, 0);
    currGamePiece =
        Shuffleboard.getTab("Match")
            .addBoolean("Game Piece", () -> robotStateSubsystem.getCurrentPiece() != GamePiece.NONE)
            .withProperties(Map.of("colorWhenFalse", "black"))
            .withSize(2, 2)
            .withPosition(5, 0);
    Shuffleboard.getTab("Match")
        .addBoolean("Is Navx Connected", () -> driveSubsystem.isNavxWorking())
        .withSize(1, 1)
        .withPosition(8, 1);
  }

  private void configurePitDashboard() {
    Shuffleboard.getTab("Pit")
        .add("LockZero", new LockZeroCommand(driveSubsystem))
        .withPosition(0, 2);
  }
  // Interlink Controller Mapping FIXME
  public enum Axis {
    RIGHT_X(1),
    RIGHT_Y(0),
    LEFT_X(2),
    LEFT_Y(5),
    TUNER(6),
    LEFT_BACK(4),
    RIGHT_BACK(3);

    public final int id;

    Axis(int id) {
      this.id = id;
    }
  }

  public enum Toggle {
    LEFT_TOGGLE(1);

    private final int id;

    Toggle(int id) {
      this.id = id;
    }
  }

  public enum InterlinkButton {
    RESET(3),
    HAMBURGER(14),
    X(15),
    UP(16),
    DOWN(17);

    private final int id;

    InterlinkButton(int id) {
      this.id = id;
    }
  }

  public enum Trim {
    LEFT_Y_POS(7),
    LEFT_Y_NEG(6),
    LEFT_X_POS(8),
    LEFT_X_NEG(9),
    RIGHT_X_POS(10),
    RIGHT_X_NEG(11),
    RIGHT_Y_POS(12),
    RIGHT_Y_NEG(13);

    private final int id;

    Trim(int id) {
      this.id = id;
    }
  }

  public void setAllianceColor(Alliance alliance) {
    this.alliance = alliance;
    allianceColor.withProperties(
        Map.of(
            "colorWhenTrue", alliance == Alliance.Red ? "red" : "blue", "colorWhenFalse", "black"));
    robotStateSubsystem.setAllianceColor(alliance);
    // fiveMeterTest.generateTrajectory(); FIXME we will need to test this eventually
    // balancepath.generateTrajectory();
    // communityToDockCommandGroup.generateTrajectory();
    // twoPieceWithDockAutoCommandGroup.generateTrajectory();
    // threePiecePath.generateTrajectory();
    // twoPieceAutoPlacePathCommandGroup.generateTrajectory();
    // bumpSideTwoPieceCommandGroup.generateTrajectory();

    /*if (autoSwitch.getAutoCommand() != null) {
      autoSwitch.getAutoCommand().generateTrajectory();
    }                                                   */

    // Flips gyro angle if alliance is red team
    if (robotStateSubsystem.getAllianceColor() == Alliance.Red) {
      driveSubsystem.setGyroOffset(Rotation2d.fromDegrees(180));
    } else {
      driveSubsystem.setGyroOffset(Rotation2d.fromDegrees(0));
    }
  }

  // public void setDisabled(boolean isDisabled) {
  //   robotStateSubsystem.setDisabled(isDisabled);
  // }

  public void updateGamePiece() {
    currGamePiece.withProperties(
        Map.of(
            "colorWhenTrue",
            robotStateSubsystem.getCurrentPiece() == GamePiece.CUBE ? "purple" : "yellow",
            "colorWhenFalse",
            "black"));
  }
}
