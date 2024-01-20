// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class Eject extends Command{
  /** Creates a new eject. */
  public Eject() {
    //sets the motors to a certain rpm to drop the note into the amp
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    //apply the voltage to the motor in this method using PID
  }
  // Called once the command ends or is interrupted.

  @Override
  public void end(boolean interrupted) {}
  // Returns true when the command should end.
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
