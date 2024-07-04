package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Effectorc.*;

import org.carlmontrobotics.subsystems.Arm;
import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

// TODO: where do we use this command?
public class RampMaxRPMDriving extends Command {
  private final IntakeShooter intakeouttake;

  public RampMaxRPMDriving(IntakeShooter intake) {
    addRequirements(this.intakeouttake = intake);
  }

  @Override
  public void initialize() {
  // outtake ramps up once the 2 distance sensors detect a note
  if(intakeouttake.intakeDetectsNote() && intakeouttake.outtakeDetectsNote()){
        intakeouttake.setMaxOuttake(1);
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeouttake.stopOuttake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !intakeouttake.intakeDetectsNote() && !intakeouttake.outtakeDetectsNote();
  }
}
