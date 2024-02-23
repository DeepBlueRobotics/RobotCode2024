package org.carlmontrobotics.commands;

import org.carlmontrobotics.Constants.IntakeShoot;
import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj2.command.Command;

public class EjectRPM extends Command {
    private final IntakeShooter intake;
    private final IntakeShooter shooter;
    public EjectRPM(IntakeShooter intake, IntakeShooter shooter) {
        this.intake = intake;
        this.shooter = shooter;
    }
    @Override
    public void initialize() {
      intake.setRPMEjectIntake();
      shooter.setRPMEjectOutake();
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
    public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    shooter.stopOutake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //TODO: see how many distance sensors therre are and modify this
    return !intake.gameDistanceSees1st() && !intake.gameDistanceSees2nd() && !shooter.gameDistanceSees1st() && !shooter.gameDistanceSees2nd();
  }
}
