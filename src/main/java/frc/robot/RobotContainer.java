// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.DriveTeleopCommand;
import frc.robot.commands.drive.XLockCommand;
import frc.robot.commands.drive.ZeroGyroCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.example.ExampleIOTalon;
import frc.robot.subsystems.example.ExampleSubsystem;
import frc.robot.subsystems.shoulder.ShoulderSubsystem;
import frc.robot.subsystems.shoulder.ShoulderTalonIO;
import frc.robot.subsystems.robotstate.RobotStateSubsystem;
import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

public class RobotContainer {

  // Grapher
  private final TelemetryService telemetryService = new TelemetryService(TelemetryController::new);

  // Subsystems
  private ExampleSubsystem exampleSubsystem;
  private ShoulderSubsystem shoulder;
  private DriveSubsystem driveSubsystem;
  private RobotStateSubsystem robotStateSubsystem;

  // OI Objects
  private final Joystick driveJoystick = new Joystick(0);

  public RobotContainer() {

    exampleSubsystem = new ExampleSubsystem(new ExampleIOTalon());
    shoulder = new ShoulderSubsystem(new ShoulderTalonIO());
    driveSubsystem = new DriveSubsystem(new Swerve(telemetryService));
    configureDriverButtonBindings();
    configureBindings();
  }

  private void configureBindings() {}

  private void configureDriverButtonBindings() {
    driveSubsystem.setDefaultCommand(

        // DRIVE
        new DriveTeleopCommand(driveJoystick, driveSubsystem, robotStateSubsystem));

    new JoystickButton(driveJoystick, InterlinkButton.RESET.id)
        .onTrue(new ZeroGyroCommand(driveSubsystem));

    new JoystickButton(driveJoystick, InterlinkButton.X.id)
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
