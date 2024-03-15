package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Effectorc.*;
import static org.carlmontrobotics.Constants.Led.*;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.AuxSystems;
import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
public class Intake extends Command {
    //intake until sees game peice or 4sec has passed
    private final Timer timer = new Timer();
    private final IntakeShooter intake;

    private double endAt = 0;
    private final double keepIntakingFor = 0.2;

    public Intake(IntakeShooter intake) {
        addRequirements(this.intake = intake);
    }

    @Override
    public void initialize() {
      intake.setRPMIntake(INTAKE_RPM+1000);
      timer.reset();
      timer.start();
      intake.setCurrentLimit(20);
    }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
    public void execute() {
      //Intake Led
      if (intake.intakeDetectsNote() && !intake.outakeDetectsNote()) {
        intake.setRPMIntake(INTAKE_SLOWDOWN_RPM);
      }
      if (intake.outakeDetectsNote() ) {
       // Timer.delay(keepIntakingFor);
        intake.setRPMIntake(0.0);
      }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    timer.stop();
    intake.resetCurrentLimit();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (intake.intakeDetectsNote() && !intake.outakeDetectsNote())
      || timer.hasElapsed(MAX_SECONDS_OVERLOAD);
  }
}
