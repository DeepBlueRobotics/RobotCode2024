package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Effectorc.*;

import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNEO extends Command {
  // intake until sees game peice or 4sec has passed
  private Timer timer = new Timer();
  private final IntakeShooter intake;

  private double endAt = 0;
  private final double keepIntakingFor = 0.2;
  int increaseAmount = 750;
  int index = 0;

  public IntakeNEO(IntakeShooter intake) {
    addRequirements(this.intake = intake);

  }

  @Override
  public void initialize() {
    //TODO: Adjust speed or add in an index
    intake.motorSetIntake(0.5);
    intake.resetCurrentLimit();
    index=0; 
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Intake Led
    
   
    
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
