package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Effectorc.*;

import org.carlmontrobotics.subsystems.Arm;
import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

// TODO: where do we use this command?
public class RampRPMAuton extends Command {
  // intake until sees game peice or 4sec has passed
  private final IntakeShooter intake;


  public RampRPMAuton(IntakeShooter intake) {
    addRequirements(this.intake = intake);
  }

  @Override
  public void initialize() {
    intake.setRPMOuttake(4000);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.getOuttakeRPM() >= 4000 && intake.getOuttakeRPM() <= 4250;
  }
}
