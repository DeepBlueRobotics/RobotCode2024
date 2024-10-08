// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

import static org.carlmontrobotics.Constants.Limelightc.INTAKE_LL_NAME;
import static org.carlmontrobotics.Constants.Limelightc.SHOOTER_LL_NAME;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  public static Robot robot;

  private RobotContainer m_robotContainer;

  @Override

  public void robotInit() {
    robot = this;
    m_robotContainer = new RobotContainer();
    // SignalLogger.start();
    // creates usb camera
    // Record both DS control and joystick data
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    FollowPathCommand.warmupCommand().schedule();
    // for (int port = 5800; port <= 5809; port++) {
    // PortForwarder.add(port, INTAKE_LL_NAME + ".local", port);
    // }
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, SHOOTER_LL_NAME + ".local", port);
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
  }

  @Override
  public void disabledInit() {
    // SignalLogger.stop();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
