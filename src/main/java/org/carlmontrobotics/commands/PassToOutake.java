// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;


import static org.carlmontrobotics.Constants.Effectorc.*;
import static org.carlmontrobotics.Constants.Led.*;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.AuxSystems;
import org.carlmontrobotics.subsystems.IntakeShooter;
import static org.carlmontrobotics.Constants.Effectorc.*;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PassToOutake extends Command {
  //pass ring from intake to outtake
  private final IntakeShooter intake;

  public PassToOutake(IntakeShooter intake) {
      this.intake = intake;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    intake.setRPMIntake(PASS_RPM);
    if (Math.abs(intake.getOutakeRPM()-RPM_TOLERANCE)<=RPM_TOLERANCE){//only move it if it wasn't already ramped up to a speed
      intake.setRPMOutake(PASS_RPM);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    // intake.stopOutake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!intake.intakeDetectsNote() && !intake.outakeDetectsNote());
  }
}
