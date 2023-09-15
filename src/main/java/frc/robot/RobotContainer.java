// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.DriveTeleopCommand;
import frc.robot.commands.drive.XLockCommand;
import frc.robot.commands.drive.ZeroGyroCommand;
import frc.robot.controllers.FlyskyJoystick;
import frc.robot.controllers.FlyskyJoystick.Button;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Extendo.ExtendoIOTalon;
import frc.robot.subsystems.Extendo.ExtendoSubsystem;
import frc.robot.subsystems.Wrist.WristEncoderIOCanandcoder;
import frc.robot.subsystems.Wrist.WristIOTalon;
import frc.robot.subsystems.Wrist.WristSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.example.ExampleIOTalon;
import frc.robot.subsystems.example.ExampleSubsystem;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
import frc.robot.subsystems.shoulder.ShoulderSubsystem;
import frc.robot.subsystems.shoulder.ShoulderTalonIO;
import org.slf4j.Logger;
import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

public class RobotContainer {

  // Grapher
  private final TelemetryService telemetryService = new TelemetryService(TelemetryController::new);

  // Subsystems
  private ExampleSubsystem exampleSubsystem;
  private DriveSubsystem driveSubsystem;
  private ShoulderSubsystem shoulder;
  private ExtendoSubsystem extendoSubsystem;
  private WristSubsystem wristSubsystem;
  private ArmSubsystem armSubsystem;
  private RobotStateSubsystem robotStateSubsystem;

  // IO Objects
  private final Joystick driveJoystick = new Joystick(0);

  private Logger logger;

  public RobotContainer() {

    exampleSubsystem = new ExampleSubsystem(new ExampleIOTalon());
    shoulder = new ShoulderSubsystem(new ShoulderTalonIO());
    driveSubsystem = new DriveSubsystem();
    configureDriverButtonBindings();
    extendoSubsystem = new ExtendoSubsystem(new ExtendoIOTalon());
    wristSubsystem = new WristSubsystem(new WristIOTalon(), new WristEncoderIOCanandcoder());
    armSubsystem = new ArmSubsystem(shoulder, extendoSubsystem, wristSubsystem);
    robotStateSubsystem = new RobotStateSubsystem(driveSubsystem, armSubsystem);
    configureBindings();
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

  private void configTelemetry() {
    exampleSubsystem.registerWith(telemetryService);
    shoulder.registerWith(telemetryService);
    driveSubsystem.registerWith(telemetryService);

    telemetryService.start();
  }

  public void zeroShoulder() {
    shoulder.zero();
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
