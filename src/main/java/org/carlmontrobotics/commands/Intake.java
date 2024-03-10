package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.IntakeShoot.*;
import static org.carlmontrobotics.Constants.Led.*;
import static org.carlmontrobotics.Constants.Arm.*;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.Led;
import org.carlmontrobotics.subsystems.IntakeShooter;
import org.carlmontrobotics.subsystems.Arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
public class Intake extends Command {
    //intake until sees game peice or 4sec has passed
    private final Timer timer = new Timer();
    private final IntakeShooter intakeShooter;
    private final Arm arm;
    public Intake(IntakeShooter intakeShooter, Arm arm) {
        this.intakeShooter = intakeShooter;
        this.arm = arm;
        addRequirements(arm);
        addRequirements(intakeShooter);
    }    
    
    @Override
    public void initialize() {
      arm.setArmTarget(INTAKE_ANGLE_RAD);
      if (arm.armAtSetpoint())
        intakeShooter.setRPMIntake(INTAKE_RPM);
      

      
      timer.reset();
      timer.start();
    }
      

  // Called every time the scheduler runs while the command is scheduled.
  @Override
    public void execute() {
      //Intake Led
      
      if (intakeShooter.intakeDetectsNote()) {
        intakeShooter.setRPMIntake(INTAKE_SLOWDOWN_RPM);
        
      }
      if (intakeShooter.outakeDetectsNote() ) {
        intakeShooter.setRPMIntake(0.0);
      }  
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeShooter.stopIntake();
    timer.stop();
    //resets to defaultColor
    

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeShooter.outakeDetectsNote();
  }
}
