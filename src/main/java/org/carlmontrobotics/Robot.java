// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

import static org.carlmontrobotics.Constants.IntakeShoot.DS_DEPTH_INCHES;
import static org.carlmontrobotics.Constants.IntakeShoot.INTAKE_DISTANCE_SENSOR_PORT;
import static org.carlmontrobotics.Constants.IntakeShoot.OUTAKE_DISTANCE_SENSOR_PORT;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.Led;
import org.carlmontrobotics.subsystems.IntakeShooter;
import static org.carlmontrobotics.Constants.IntakeShoot.*;



public class Robot extends TimedRobot {
  private final Led led = new Led();
  private final IntakeShooter intakeShoot = new IntakeShooter();

  private Command m_autonomousCommand;
  public static Robot robot;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    robot = this;
//  //for(int i = 0; i<100; i++){
//      TimeOfFlight intakeDistanceSensor1 = new TimeOfFlight(i); // make sure id port is correct here
//      double distance = Units.metersToInches((intakeDistanceSensor1.getRange() - DS_DEPTH_INCHES) / 1000);
//      if(distance > 0.2){
//       System.out.println("something");
//       System.out.println(i);
//      }

 //}
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

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
