// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.DriveTeleopCommand;
import frc.robot.commands.drive.XLockCommand;
import frc.robot.commands.drive.ZeroGyroCommand;
import frc.robot.commands.robotState.ClearGamepieceCommand;
import frc.robot.commands.robotState.FloorPickupCommand;
import frc.robot.commands.robotState.ManualStageArmCommand;
import frc.robot.commands.robotState.ReleaseGamePieceCommand;
import frc.robot.commands.robotState.SetTargetLevelCommand;
import frc.robot.commands.robotState.StowCommand;
import frc.robot.commands.robotState.SubstationPickupCommand;
import frc.robot.commands.shoulder.ZeroShoulderCommand;
import frc.robot.controllers.FlyskyJoystick;
import frc.robot.controllers.FlyskyJoystick.Button;
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
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

public class RobotContainer {

  // Grapher
  private final TelemetryService telemetryService = new TelemetryService(TelemetryController::new);

  // Subsystems
  private ExampleSubsystem exampleSubsystem;
  private DriveSubsystem driveSubsystem;
  private MinimalShoulderSubsystem shoulderSubsystem;
  private HandSubsystem handSubsystem;
  private MinimalRobotStateSubsystem robotStateSubsystem;

  // IO Objects
  private final Joystick driveJoystick = new Joystick(0);
  private final XboxController xboxController = new XboxController(1);

  private Logger logger = LoggerFactory.getLogger(RobotContainer.class);

  public RobotContainer() {

    exampleSubsystem = new ExampleSubsystem(new ExampleIOTalon());
    shoulderSubsystem = new MinimalShoulderSubsystem(new MinimalShoulderFalconIO());
    handSubsystem = new HandSubsystem(new HandIOFalcon());
    robotStateSubsystem =
        new MinimalRobotStateSubsystem(driveSubsystem, shoulderSubsystem, handSubsystem);
    driveSubsystem = new DriveSubsystem();
    driveSubsystem.setRobotStateSubsystem(robotStateSubsystem);

    configureDriverButtonBindings();
    configureOperatorBindings();

    configTelemetry();

    driveSubsystem.teleResetGyro();
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
        .onTrue(new ClearGamepieceCommand(robotStateSubsystem));

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
        .onTrue(new ReleaseGamePieceCommand(robotStateSubsystem, handSubsystem));

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
}
