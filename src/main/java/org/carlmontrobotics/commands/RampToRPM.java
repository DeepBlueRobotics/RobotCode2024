package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Effectorc.*;

import org.carlmontrobotics.subsystems.Arm;
import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

// TODO: where do we use this command?
public class RampToRPM extends Command {
  // intake until sees game peice or 4sec has passed
  private final double rpm;
  private final IntakeShooter intake;
  private Timer timer;

  public RampToRPM(IntakeShooter intake) {
    addRequirements(this.intake = intake);
    rpm = RPM_SELECTOR[Arm.getSelector()];
  }

  @Override
  public void initialize() {
    intake.setRPMOutake(rpm);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopOutake();
    // resets to defaultColor
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
