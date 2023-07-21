// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.example.ExampleIOTalon;
import frc.robot.subsystems.example.ExampleSubsystem;
import frc.robot.subsystems.shoulder.ShoulderSubsystem;
import frc.robot.subsystems.shoulder.ShoulderTalonIO;
import frc.robot.subsystems.robotState.RobotStateSubsystem;
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

  // IO Objects

  public RobotContainer() {

    exampleSubsystem = new ExampleSubsystem(new ExampleIOTalon());
    shoulder = new ShoulderSubsystem(new ShoulderTalonIO());
    driveSubsystem = new DriveSubsystem();
    robotStateSubsystem =
        new RobotStateSubsystem(driveSubsystem); // TODO: add inputs for other subsystems
    configureBindings();
  }

  private void configureBindings() {}

  private void configTelemetry() {
    exampleSubsystem.registerWith(telemetryService);
    shoulder.registerWith(telemetryService);
    telemetryService.start();
  }

  public void zeroShoulder() {
    shoulder.zero();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
