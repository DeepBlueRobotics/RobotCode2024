package org.carlmontrobotics.commands;

import org.carlmontrobotics.Constants.IntakeShoot;
import org.carlmontrobotics.subsystems.IntakeShooter;
import org.carlmontrobotics.Constants;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterToRPM extends Command {
    //ramp the shooter up to a specific rpm
    private final IntakeShooter shooter;
    private double outakeRPM;
    public ShooterToRPM(IntakeShooter shooter, double rpm) {
        this.shooter = shooter;
        this.outakeRPM = rpm;
    }

    @Override
    public void initialize() {
      shooter.setRPMOutake(outakeRPM);
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
    public void execute() {
    // use trapazoid math and controllerMoveArm method from arm subsytem to apply voltage to the motor
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.isWithenTolerance(outakeRPM);
  }
}
