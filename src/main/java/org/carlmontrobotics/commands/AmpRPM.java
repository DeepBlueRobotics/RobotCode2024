// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;


import static org.carlmontrobotics.Constants.IntakeShoot.*;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.IntakeShooter;
import static org.carlmontrobotics.Constants.IntakeShoot.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AmpRPM extends Command {
  //pass ring from intake to outtake
  private final IntakeShooter intakeShooter;
  private Timer timer=new Timer();

  public AmpRPM(IntakeShooter intake) {
      this.intakeShooter = intake;
      addRequirements(intake);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    timer.reset();
      intakeShooter.setRPMIntake(AMP_RPM);
      intakeShooter.setRPMOutake(AMP_RPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeShooter.setRPMOutake(AMP_RPM);
    if(intakeShooter.isWithinTolerance()){
      intakeShooter.setMaxIntake(-1);
      timer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeShooter.stopIntake();
    intakeShooter.stopOutake();
    intakeShooter.setCurrentLimit(20);
    timer.stop();
    //resets to defaultColor
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!intakeShooter.intakeDetectsNote() && !intakeShooter.outakeDetectsNote()) || timer.get()>1.5;
  }
}
