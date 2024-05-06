package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Effectorc.*;

import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNEO extends Command {
  // intake until sees game peice or 4sec has passed
  private Timer timer = new Timer();
  private final IntakeShooter intake;
  double increaseAmount = 0.05;
  int index = 0;
  public int speed;

  public IntakeNEO(IntakeShooter intake) {
    addRequirements(this.intake = intake);
    SmartDashboard.putNumber("Intake RPM", speed);

  }

  @Override
  public void initialize() {
    //TODO: Adjust speed or add in an index
    timer.reset();
    // if (intake.intakeDetectsNote()) {
    // return;
    // }
    intake.motorSetIntake(SmartDashboard.getNumber("Intake RPM", speed));

    intake.resetCurrentLimit();
    index=0; 
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.motorSetIntake(SmartDashboard.getNumber("Intake RPM", speed));
    // Intake Led
    if((intake.intakeDetectsNote())) {
      timer.start();
    } else {
      timer.stop();
      timer.reset();
    }
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
    // return intake.intakeDetectsNote() && timer.get()>0.1;
    // || //timer.hasElapsed(MAX_SECONDS_OVERLOAD);
    return intake.intakeDetectsNote();
  }
}
