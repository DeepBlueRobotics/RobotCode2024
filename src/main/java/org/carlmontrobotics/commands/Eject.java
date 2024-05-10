package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Armc.SMART_CURRENT_LIMIT_TIMEOUT;

import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class Eject extends Command {
  // eject until no more game peice
  private final IntakeShooter intakeShooter;

  private final Timer timer = new Timer();

  public Eject(IntakeShooter intakeShooter) {
    addRequirements(this.intakeShooter = intakeShooter);
  }

  @Override
  public void initialize() {
    // intakeShooter.setRPMIntake(EJECT_RPM_INTAKE);
    // intakeShooter.setRPMOutake(EJECT_RPM_OUTAKE);
    timer.reset();
    timer.start();
    intakeShooter.setMaxOuttake(-1);
    intakeShooter.setMaxIntake(-1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeShooter.stopIntake();
    intakeShooter.stopOuttake();
    timer.stop();
    intakeShooter.resetCurrentLimit();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() > SMART_CURRENT_LIMIT_TIMEOUT
        || (!intakeShooter.intakeDetectsNote() && !intakeShooter.outtakeDetectsNote()));
  }
}
