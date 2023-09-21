// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.robotState.SetAllianceCommand;
import frc.robot.constants.BuildConstants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private boolean haveAlliance;

  @Override
  public void robotInit() {
    Logger logger = LoggerFactory.getLogger(Robot.class);
    org.littletonrobotics.junction.Logger advLogger =
        org.littletonrobotics.junction.Logger.getInstance();

    // Record metadata
    advLogger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    advLogger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    advLogger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    advLogger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    advLogger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        advLogger.recordMetadata("GitDirty", "All Changes Committed");
        break;
      case 1:
        advLogger.recordMetadata("GitDirty", "Uncommitted changes");
        break;
      default:
        advLogger.recordMetadata("GitDirty", "Unknown");
        break;
    }
    if (RobotBase.isReal()) {
      advLogger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to USB stick
      advLogger.addDataReceiver(new NT4Publisher()); // Publish data to Network Tables
    } else {
      setUseTiming(false);
      String logPath = LogFileUtil.findReplayLog(); // pull replay log from advantage scope
      advLogger.setReplaySource(new WPILOGReader(logPath));
      advLogger.addDataReceiver(
          new WPILOGWriter(
              LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // Start Advantage kit logger
    advLogger.start();

    m_robotContainer = new RobotContainer();

    haveAlliance = false;

    Shuffleboard.getTab("Match")
        .add("SetAllianceRed", new SetAllianceCommand(Alliance.Red, m_robotContainer))
        .withPosition(2, 0)
        .withSize(1, 1);

    Shuffleboard.getTab("Match")
        .add("SetAllianceBlue", new SetAllianceCommand(Alliance.Blue, m_robotContainer))
        .withPosition(2, 1)
        .withSize(1, 1);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.updateGamePiece();

    if (!haveAlliance) {
      Alliance alliance = DriverStation.getAlliance();
      if (alliance != Alliance.Invalid) {
        haveAlliance = true;
        m_robotContainer.setAllianceColor(alliance);
        // logger.info("Set Alliance {}", alliance);
        // m_robotContainer.getAutoSwitch().getAutoCommand().generateTrajectory();
      }
    }
  }

  @Override
  public void disabledInit() {
    // logger.info("Disabled Init");
    m_robotContainer.setDisabled(true);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
