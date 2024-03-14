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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PassToOutake extends Command {
  //pass ring from intake to outtake
  private final IntakeShooter intake;
  Timer timer = new Timer();

  public PassToOutake(IntakeShooter intake) {
      addRequirements(this.intake = intake);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    timer.reset();
    intake.setMaxOutake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //intake.setMaxOutake();
    if (intake.getOutakeRPM()>=6000) {//SPEAKER_RPM){
      intake.setMaxIntake(1);
      timer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    intake.resetCurrentLimit();
    timer.stop();
    intake.stopOutake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!intake.intakeDetectsNote() && !intake.outakeDetectsNote()) || timer.get()>1.5;
  }
}
