// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeShoot extends Command {
  /** Creates a new Shoot. */
  public void Shoot() {
    //Sets the motors to a certain rpm to shoot into the speaker when the right trigger is pressed(need PID)
  }

  public void Intake() {
    //Sets the motors to a certain rpm to intake a note when the left trigger is pressed
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.(Periodic)
  
  @Override
  public void execute() {
      //apply the voltage to the motor in this method using PID for shooter
      //apply the voltage to the motor here for Intake
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
