package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Effectorc.*;

import org.carlmontrobotics.Constants;

import org.carlmontrobotics.subsystems.IntakeShooter;
import org.carlmontrobotics.subsystems.Led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
public class Intake extends Command {
  // intake until sees game peice or 4sec has passed
  private Timer timer = new Timer();
  private final IntakeShooter intake;

  private double endAt = 0;
  private final double keepIntakingFor = 0.2;
  int increaseAmount = 750;
  int index = 0;

  public Intake(IntakeShooter intake) {
    addRequirements(this.intake = intake);

  }

  @Override
  public void initialize() {
    intake.setRPMIntake(INTAKE_RPM);
    intake.resetCurrentLimit();
    index=0; 
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Intake Led
    
    if (intake.intakeDetectsNote() && !intake.outakeDetectsNote()) {
      index++;

      //intake.setRPMIntake(0);
     intake.setRPMIntake(INTAKE_RPM + index*increaseAmount);
    }
    if (intake.outakeDetectsNote()) {
      // Timer.delay(keepIntakingFor);

      intake.setRPMIntake(0.0);
    }
    if(!intake.intakeDetectsNote()) {
      intake.setRPMIntake(INTAKE_RPM);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    timer.stop();
    index = 0;
    //intake.resetCurrentLimit();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (intake.intakeDetectsNote() && intake.outakeDetectsNote());
    // || //timer.hasElapsed(MAX_SECONDS_OVERLOAD);

  }
}
