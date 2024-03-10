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
    
    public Intake(IntakeShooter intake) {
        addRequirements(this.intake = intake);
    }    
    
    @Override
    public void initialize() {
      intake.setRPMIntake(INTAKE_RPM);
      intake.stopOutake();
      timer.reset();
      timer.start();
    }
      

  // Called every time the scheduler runs while the command is scheduled.
  @Override
    public void execute() {
      //Intake Led
      if (intake.intakeDetectsNote() && !intake.outakeDetectsNote()) {
        intake.setRPMIntake(INTAKE_SLOWDOWN_RPM);
      }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (intake.intakeDetectsNote() && !intake.outakeDetectsNote()) 
      || timer.hasElapsed(INTAKE_TIME_SECS);
  }
}
