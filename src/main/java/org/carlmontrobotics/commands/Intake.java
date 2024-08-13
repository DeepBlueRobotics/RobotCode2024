package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Effectorc.*;

import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends Command {
  // intake until sees game peice or 4sec has passed
  private final IntakeShooter intake;
  private int index = 0;
  private double increaseSpeed = .01;
  private double initSpeed = .5;
  private double slowSpeed = .1;

  public Intake(IntakeShooter intake) {
    addRequirements(this.intake = intake);
    SmartDashboard.putNumber("Initial intake speed", initSpeed);
    SmartDashboard.putNumber("Slow intake speed", slowSpeed);
    SmartDashboard.putNumber("Increase speed", increaseSpeed);
  }
  // .-18

  @Override
  public void initialize() {
    // TODO: Adjust speed or add in an index;
    // if (intake.intakeDetectsNote()) {
    // return;
    // }
    initSpeed = SmartDashboard.getNumber("Initial intake speed", 0);
    intake.motorSetIntake(initSpeed); // Fast intake speed
                                      // for initial
                                      // intake
    intake.resetCurrentLimit();
    index = 0;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Intake Led
    increaseSpeed = SmartDashboard.getNumber("Increase speed", 0);
    slowSpeed = SmartDashboard.getNumber("Slow intake speed", 0);
    if ((intake.intakeDetectsNote())) {
      double intakeSpeed = slowSpeed + index * increaseSpeed;
      SmartDashboard.putNumber("Intake-current-speed", intakeSpeed);
      intake.motorSetIntake(intakeSpeed); // Slower intake
                                          // speed triggered
                                          // after intake ds
                                          // sees note
      ++index;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    // intake.resetCurrentLimit();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return intake.intakeDetectsNote() && timer.get()>0.1;
    // || //timer.hasElapsed(MAX_SECONDS_OVERLOAD);
    return intake.outtakeDetectsNote();
  }
}
