package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.IntakeShoot.INTAKE_RPM;

import org.carlmontrobotics.Constants.IntakeShoot;
import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends Command {
    //intake until sees game peice or 4sec has passed
    private final IntakeShooter intake;
    public Intake(IntakeShooter intake) {
        this.intake = intake;
    }
    @Override
    public void initialize() {
      intake.setRPMintake(INTAKE_RPM);
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
    public void execute() {
    // use trapazoid math and controllerMoveArm method from arm subsytem to apply voltage to the motor
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.gameDistanceSees1st();
  }
}
