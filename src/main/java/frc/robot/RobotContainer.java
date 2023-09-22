// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.TestBalanceCommand;
import frc.robot.commands.drive.DriveAutonCommand;
import frc.robot.commands.drive.DriveTeleopCommand;
import frc.robot.commands.drive.XLockCommand;
import frc.robot.commands.drive.ZeroGyroCommand;
import frc.robot.controllers.FlyskyJoystick;
import frc.robot.controllers.FlyskyJoystick.Button;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.autoSwitch.AutoSwitch;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.example.ExampleIOTalon;
import frc.robot.subsystems.example.ExampleSubsystem;
import frc.robot.subsystems.extendo.ExtendoIOTalon;
import frc.robot.subsystems.extendo.ExtendoSubsystem;
import frc.robot.subsystems.hand.HandIOFalcon;
import frc.robot.subsystems.hand.HandSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.shoulder.ShoulderSubsystem;
import frc.robot.subsystems.shoulder.ShoulderTalonIO;
import frc.robot.subsystems.wrist.WristEncoderIOCanandcoder;
import frc.robot.subsystems.wrist.WristIOTalon;
import frc.robot.subsystems.wrist.WristSubsystem;
import java.util.Map;
import org.slf4j.Logger;
import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

public class RobotContainer {
  // field data
  private SuppliedValueWidget<Boolean> allianceColor;
  private Alliance alliance = Alliance.Invalid;

  // Grapher
  private final TelemetryService telemetryService = new TelemetryService(TelemetryController::new);

  // Subsystems
  private final ExampleSubsystem exampleSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final ShoulderSubsystem shoulderSubsystem;
  private final ExtendoSubsystem extendoSubsystem;
  private final WristSubsystem wristSubsystem;
  private final ArmSubsystem armSubsystem;
  private final RobotStateSubsystem robotStateSubsystem;
  private final HandSubsystem handSubsystem;
  private final AutoSwitch autoSwitch;

  // IO Objects
  private final Joystick driveJoystick = new Joystick(0);

  // auton stuff
  private TestBalanceCommand balancepath;
  private DriveAutonCommand fiveMeterTest;

  private Logger logger;

  public RobotContainer() {

    exampleSubsystem = new ExampleSubsystem(new ExampleIOTalon());
    shoulderSubsystem = new ShoulderSubsystem(new ShoulderTalonIO());
    driveSubsystem = new DriveSubsystem(new Swerve(telemetryService));
    configureDriverButtonBindings();
    extendoSubsystem = new ExtendoSubsystem(new ExtendoIOTalon());
    wristSubsystem = new WristSubsystem(new WristIOTalon(), new WristEncoderIOCanandcoder());
    armSubsystem = new ArmSubsystem(shoulderSubsystem, extendoSubsystem, wristSubsystem);
    handSubsystem = new HandSubsystem(new HandIOFalcon());
    robotStateSubsystem = new RobotStateSubsystem(driveSubsystem, armSubsystem);
    configureBindings();

    autoSwitch = new AutoSwitch(robotStateSubsystem, driveSubsystem, armSubsystem, handSubsystem);
  }

  public void setAllianceColor(Alliance alliance) {
    this.alliance = alliance;
    allianceColor.withProperties(
        Map.of(
            "colorWhenTrue", alliance == Alliance.Red ? "red" : "blue", "colorWhenFalse", "black"));
    robotStateSubsystem.setAllianceColor(alliance);
    fiveMeterTest.generateTrajectory();
    balancepath.generateTrajectory();
    // communityToDockCommandGroup.generateTrajectory();
    // twoPieceWithDockAutoCommandGroup.generateTrajectory();
    // threePiecePath.generateTrajectory();
    // twoPieceAutoPlacePathCommandGroup.generateTrajectory();
    // bumpSideTwoPieceCommandGroup.generateTrajectory();
    if (autoSwitch.getAutoCommand() != null) {
      autoSwitch.getAutoCommand().generateTrajectory();
    }
    // Flips gyro angle if alliance is red team
    if (robotStateSubsystem.getAllianceColor() == Alliance.Red) {
      driveSubsystem.setGyroOffset(Rotation2d.fromDegrees(180));
    } else {
      driveSubsystem.setGyroOffset(Rotation2d.fromDegrees(0));
    }
  }

  private void configureBindings() {}

  public boolean configureDriverButtonBindings() {
    String joystick = DriverStation.getJoystickName(0);
    boolean success = false;
    switch (joystick) {
      case "InterLink-X":
        logger.info("Configuring Interlink Joystick");
        configureInterlinkDriverButtonBindings();
        success = true;
        break;
      case "FlySky NV14 Joystick":
        logger.info("Configuring Flysky Joystick");
        configureFlyskyDriverButtonBindings();
        success = true;
        break;
      default:
        logger.info("No joystick type {} defined", joystick);
        break;
    }
    return success;
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
            robotStateSubsystem));

    new JoystickButton(driveJoystick, Button.M_LTRIM_UP.id)
        .onTrue(new ZeroGyroCommand(driveSubsystem));

    new JoystickButton(driveJoystick, Button.M_RTRIM_UP.id)
        .onTrue(new XLockCommand(driveSubsystem));
  }

  private void configureMatchDashboard() {
    allianceColor =
        Shuffleboard.getTab("Match")
            .addBoolean("AllianceColor", () -> alliance != Alliance.Invalid)
            .withProperties(Map.of("colorWhenFalse", "black"))
            .withSize(2, 2)
            .withPosition(0, 0);
  }

  private void configTelemetry() {
    exampleSubsystem.registerWith(telemetryService);
    shoulderSubsystem.registerWith(telemetryService);
    driveSubsystem.registerWith(telemetryService);
    telemetryService.start();
  }

  public void zeroShoulder() {
    shoulderSubsystem.zero();
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
