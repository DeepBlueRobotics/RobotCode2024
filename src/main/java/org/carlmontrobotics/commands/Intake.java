// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.             
//TODO: consider making intake semi autonomous 
package org.carlmontrobotics.commands;
import org.carlmontrobotics.subsystems.IntakeShooter;

import com.playingwithfusion.TimeOfFlight;

import static org.carlmontrobotics.Constants.IntakeShooter.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake extends Command {
  private final IntakeShooter InShoot;
  /** Creates a new Intake. */

  public Intake(IntakeShooter InShoot) {
    this.InShoot = InShoot;
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
    //will check if the is holding note boolean is true
  }
}
