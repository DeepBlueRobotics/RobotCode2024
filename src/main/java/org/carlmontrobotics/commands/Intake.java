package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.IntakeShoot.*;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
public class Intake extends Command {
    //intake until sees game peice or 4sec has passed
    private final Timer timer = new Timer();
    //private double sawAt=0.;
    private final IntakeShooter intake;
    public Intake(IntakeShooter intake) {
        this.intake = intake;
        addRequirements(intake);
    }    
    
    @Override
    public void initialize() {
      intake.setRPMIntake(INTAKE_RPM);
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
      if (intake.outakeDetectsNote() ) {
        intake.setRPMIntake(0.0);
        // if (sawAt==0) {
        //   sawAt = Timer.getFPGATimestamp();
        // }
      }  
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    timer.stop();
    //resets to defaultColor
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.outakeDetectsNote();
  }
}
