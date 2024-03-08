// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;


import static org.carlmontrobotics.Constants.IntakeShoot.*;
import static org.carlmontrobotics.Constants.Led.*;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.Led;
import org.carlmontrobotics.subsystems.IntakeShooter;
import static org.carlmontrobotics.Constants.IntakeShoot.*;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PassToOutake extends Command {
  //pass ring from intake to outtake
  private final IntakeShooter intake;
  private final Led led = new Led();

  public PassToOutake(IntakeShooter intake) {
      this.intake = intake;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    new SequentialCommandGroup(
      new InstantCommand(() -> intake.setRPMOutake(SPEAKER_RPM)),
      new InstantCommand(() -> intake.stopIntake())
      ).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intake.isWithinTolerance()){
      intake.setRPMIntake(PASS_RPM);
    }
    //Shooting Led
    if (!intake.intakeDetectsNote() && !intake.outakeDetectsNote()) {
      led.setLedColor(outtakeColor, 0, led.Midpoint);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    intake.stopOutake();
    intake.setRumblyTumbly(false);
    //resets to defaultColor
    led.setLedColor(Constants.Led.defaultColor, 0, led.Midpoint);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !intake.intakeDetectsNote();
  }
}
