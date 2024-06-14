package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Effectorc.*;
import org.carlmontrobotics.subsystems.IntakeShooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeAutonRuiner extends Command {
  // intake until sees game peice or 4sec has passed
  private Timer timer = new Timer();
  private final IntakeShooter intake;
  int index = 0;

  public IntakeAutonRuiner(IntakeShooter intake) {
    addRequirements(this.intake = intake);

  }

  @Override
  public void initialize() {
    intake.setRPMIntake(INTAKE_RPM);
    timer.reset();
    timer.start();
    intake.resetCurrentLimit();
    index = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Intake Led
    if (intake.intakeDetectsNote() && !intake.outtakeDetectsNote()) {
      index++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    timer.stop();
    index = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 15;

  }
}
