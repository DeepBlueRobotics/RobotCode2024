// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.             
//TODO: also need to work with arm
//TODO: consider making intake semi autonomous 
//TODO: change name to AMP
package org.carlmontrobotics.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends Command { 

  /** Creates a new Intake. */

  //TODO: pass in the IntakeShooter subsystem
  public Intake() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //calls the setRPMIntake method in the IntakeShooter subsystem when method is ran
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //placeholder
    return true;
    //should check if have ring or 5sec passed
  }
}
