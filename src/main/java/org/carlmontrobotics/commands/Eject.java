package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.IntakeShoot.*;

import org.carlmontrobotics.subsystems.IntakeShooter;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;


public class Eject extends Command {
    //eject until no more game peice
    //raise arm while setting motors to speed, if motor at speed and arm at goal pos then shoot
    private final IntakeShooter intakeShooter;

    private final Timer timer = new Timer();
    public Eject(IntakeShooter intakeShooter) {
        this.intakeShooter = intakeShooter;

        addRequirements(intakeShooter);
    
    }
    @Override
    public void initialize() 
    {
      intakeShooter.setRPMOutake(EJECT_RPM_OUTAKE);
      
      if(intakeShooter.isWithinTolerance())
      {
        intakeShooter.setRPMIntake(EJECT_RPM_INTAKE);
        
      }
      

      timer.reset();
      timer.start();
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
    public void execute() 
    {
      
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    intakeShooter.stopIntake();
    intakeShooter.stopOutake();
    timer.stop();
    intakeShooter.setRumblyTumbly(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !intakeShooter.intakeDetectsNote() && !intakeShooter.outakeDetectsNote()|| timer.hasElapsed(EJECT_TIME_SECS) ;
  }
}
