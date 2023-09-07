// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  public RobotContainer() {

    exampleSubsystem = new ExampleSubsystem(new ExampleIOTalon());
    driveSubsystem = new DriveSubsystem();
    shoulder = new ShoulderSubsystem(new ShoulderTalonIO());
    extendoSubsystem = new ExtendoSubsystem(new ExtendoIOTalon());
    wristSubsystem = new WristSubsystem(new WristIOTalon(), new WristEncoderIOCanandcoder());
    armSubsystem = new ArmSubsystem(shoulder, extendoSubsystem, wristSubsystem);
    robotStateSubsystem = new RobotStateSubsystem(driveSubsystem, armSubsystem);
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
