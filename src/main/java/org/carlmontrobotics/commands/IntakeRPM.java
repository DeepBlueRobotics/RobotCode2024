package org.carlmontrobotics.commands;

import org.carlmontrobotics.Constants.IntakeShoot;
import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeRPM extends Command {
    private final IntakeShooter intake;
    public IntakeRPM(IntakeShooter intake) {
        this.intake = intake;
    }
    @Override
    public void initialize() {
      intake.setRPMintake();
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
    //TODO: see how many distance sensors therre are and modify this
    return !intake.gameDistanceSees1st() && !intake.gameDistanceSees2nd();
  }
}
