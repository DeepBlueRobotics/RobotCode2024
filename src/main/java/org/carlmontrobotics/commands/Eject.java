package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Effectorc.*;

import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

public class Eject extends Command {
    //eject until no more game peice
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
      intakeShooter.setMaxOutakeOverload(1);
      intakeShooter.setMaxIntake(-1);
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
    public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeShooter.stopIntake();
    intakeShooter.stopOutake();
    timer.stop();
    intakeShooter.resetCurrentLimit();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.hasElapsed(INTAKE_TIME_SECS-2) && !intakeShooter.intakeDetectsNote() && !intakeShooter.outakeDetectsNote());
  }
}
