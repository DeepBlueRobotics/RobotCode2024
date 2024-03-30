package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Effectorc.INTAKE_RPM;
import static org.carlmontrobotics.Constants.Effectorc.INTAKE_SLOWDOWN_RPM;

import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

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
    timer.reset();
    timer.start();
    intake.resetCurrentLimit();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Intake Led
    if (intake.intakeDetectsNote() && !intake.outakeDetectsNote()) {
      //index++;

      //intake.setRPMIntake(0);
     intake.setRPMIntake(0);
    }
    if (intake.outakeDetectsNote()) {
      // Timer.delay(keepIntakingFor);

      intake.setRPMIntake(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    timer.stop();
    index= 0;
    //intake.resetCurrentLimit();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (intake.intakeDetectsNote() && intake.outakeDetectsNote());
    // || //timer.hasElapsed(MAX_SECONDS_OVERLOAD);

  }
}
