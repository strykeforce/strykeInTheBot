// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.example.ExampleIOTalon;
import frc.robot.subsystems.example.ExampleSubsystem;
import frc.robot.subsystems.robotState.MinimalRobotStateSubsystem;
import frc.robot.subsystems.shoulder.MinimalShoulderFalconIO;
import frc.robot.subsystems.shoulder.MinimalShoulderSubsystem;
import org.strykeforce.telemetry.TelemetryController;
import org.strykeforce.telemetry.TelemetryService;

public class RobotContainer {

  // Grapher
  private final TelemetryService telemetryService = new TelemetryService(TelemetryController::new);

  // Subsystems
  private ExampleSubsystem exampleSubsystem;
  private DriveSubsystem driveSubsystem;
  private MinimalShoulderSubsystem shoulder;
  private MinimalRobotStateSubsystem robotStateSubsystem;

  // IO Objects

  public RobotContainer() {

    exampleSubsystem = new ExampleSubsystem(new ExampleIOTalon());
    driveSubsystem = new DriveSubsystem();
    shoulder = new MinimalShoulderSubsystem(new MinimalShoulderFalconIO());
    robotStateSubsystem = new MinimalRobotStateSubsystem(driveSubsystem, shoulder);
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
