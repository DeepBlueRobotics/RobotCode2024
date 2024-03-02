package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.IntakeShoot.*;
//import org.carlmontrobotics.Constants.IntakeShoot.EJECT_RPM_INTAKE;

import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

public class EjectRPM extends Command {
    //eject until no more game peice
    private final IntakeShooter intakeShooter;
    private final Timer timer = new Timer();
    public EjectRPM(IntakeShooter intakeShooter) {
        this.intakeShooter = intakeShooter;
    }
    @Override
    public void initialize() {
      intakeShooter.setRPMintake(EJECT_RPM_INTAKE);
      intakeShooter.setRPMOutake(EJECT_RPM_OUTAKE);
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
    public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeShooter.stopIntake();
    intakeShooter.stopOutake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeShooter.noNote()|| timer.hasElapsed(5);
  }
}
