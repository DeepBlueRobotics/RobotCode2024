// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//TODO: consider making shoot semi autonomous - limelight
package org.carlmontrobotics.commands;
import org.carlmontrobotics.subsystems.IntakeShooter;
import edu.wpi.first.wpilibj2.command.Command;

public class Speaker extends Command {
  private final IntakeShooter InShoot;

  //TODO: all commands have to work with arm
  //TODO: write javadoc comments for all methods and commands
  /** Creates a new Shoot. */
  public Speaker(IntakeShooter InShoot) {
    addRequirements(this.InShoot = InShoot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //calls the setRPMSpeaker method in the IntakeShooter subsystem when method is ran
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
    return true;
    //we will use timer and wait 2 seconds since the right trigger is pressed
  }
}