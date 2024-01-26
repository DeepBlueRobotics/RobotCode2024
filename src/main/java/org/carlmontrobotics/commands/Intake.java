// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.             
//TODO: consider making intake semi autonomous 
package org.carlmontrobotics.commands;
import org.carlmontrobotics.subsystems.IntakeShooter;
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
    //should check if we have ring or 5 seconds has passed
    
    /*/probably need 1 If statement and 1 If else statement.    
    1. One to check if the motor speed or compression is different than the regular speed.  
       The If part of the statement will return true while the else part of the 
       statement will return false.  If the statement is true, then the Intake timer will stop
       and reset
    2. This is a backup if statement if the motor speed if statement is not checking. 
       If 5 seconds has passed since pressing the left trigger and the other If statement hasn't 
       responded, then we assume that the note has been intaked and the code returns true.   
    /*/
  }
}
