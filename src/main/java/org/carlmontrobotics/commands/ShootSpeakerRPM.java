package org.carlmontrobotics.commands;

import org.carlmontrobotics.Constants.IntakeShoot;
import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ShootSpeakerRPM extends Command {
    private final IntakeShooter shooter;
    public ShootSpeakerRPM(IntakeShooter shooter) {
        this.shooter = shooter;
    }
    @Override
    public void initialize() {
      shooter.setRPMOutake(6000);
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
    public void execute() {
    
    // use trapazoid math and controllerMoveArm method from arm subsytem to apply voltage to the motor
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopOutake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
