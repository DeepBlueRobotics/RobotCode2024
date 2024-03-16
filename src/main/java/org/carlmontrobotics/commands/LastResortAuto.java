// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

//import static org.carlmontrobotics.Constants.Effectorc.MAX_SECONDS_DRIVE;
import static org.carlmontrobotics.Constants.Effectorc.MAX_SECONDS_OVERLOAD;

import java.sql.Time;

import org.carlmontrobotics.subsystems.Arm;
import org.carlmontrobotics.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.Constants.Effectorc;



public class LastResortAuto extends Command {
  private final Timer timer = new Timer();
  /** Creates a new LastResortAuto. */
  private final Drivetrain drivetrain;
  private boolean prev;
 
 int MAX_SECONDS_DRIVE = 4;
  public LastResortAuto(Drivetrain drivetrain) {
    addRequirements(this.drivetrain = drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    prev = drivetrain.getFieldOriented();
    drivetrain.setFieldOriented(false);
    drivetrain.drive(1, 0, 0);
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setFieldOriented(prev);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(4);
  }
}
