// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeShoot extends Command {
  /** Creates a new Shoot. */
  public void Shoot() {
    // Tells the motors to spin when the button is pressed
  }

  public void Intake() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void isHoldingNote(){
    // method to check if holding note (beambreaker / digital input)
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
