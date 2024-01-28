// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private TimeOfFlight distSensor = new TimeOfFlight(10);
  private double DSdepth = 9.97;
  private double DSdetectdistance = 14;
  private CANSparkMax motor = MotorControllerFactory.createSparkMax(9, MotorConfig.NEO);

  public boolean hasGamePiece() {
    // return false;
    return getGamePieceDistanceIn() < DSdetectdistance;
  }

  public double getGamePieceDistanceIn() {
    return Units.metersToInches((distSensor.getRange() - DSdepth) / 1000 /* Convert mm to m */);
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    SmartDashboard.putNumber("motor voltage", 0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Detect Distance", getGamePieceDistanceIn());
    SmartDashboard.putBoolean("Has GUM", hasGamePiece());
    if (hasGamePiece()) motor.set(0);
    else motor.set(SmartDashboard.getNumber("motor voltage", 0));
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
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
