package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Effectorc.*;

import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNEO extends Command {
  // intake until sees game peice or 4sec has passed
  private final IntakeShooter intake;

  public IntakeNEO(IntakeShooter intake) {
    addRequirements(this.intake = intake);
  }

  @Override
  public void initialize() {
    // TODO: Adjust speed or add in an index;
    // if (intake.intakeDetectsNote()) {
    // return;
    // }
    intake.motorSetIntake(.5); // Fast intake speed for initial intake

    intake.resetCurrentLimit();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Intake Led
    if((intake.intakeDetectsNote())) {
      intake.motorSetIntake(.3); // Slower intake speed triggered after intake ds sees note
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    //intake.resetCurrentLimit();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return intake.intakeDetectsNote() && timer.get()>0.1;
    // || //timer.hasElapsed(MAX_SECONDS_OVERLOAD);
    return intake.outtakeDetectsNote();
  }
}
