package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.IntakeShoot.*;

import org.carlmontrobotics.subsystems.IntakeShooter;
import org.carlmontrobotics.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import static org.carlmontrobotics.Constants.Arm.*;

public class Eject extends Command {
    //eject until no more game peice
    // raise arm while setting motors to speed, if motor at speed and arm at goal pos then shoot
    private final IntakeShooter intakeShooter;
    private final Arm arm = new Arm();
    private final Timer timer = new Timer();
    public Eject(IntakeShooter intakeShooter) {
        this.intakeShooter = intakeShooter;
    }
    @Override
    public void initialize() {
      intakeShooter.setRPMOutake(EJECT_RPM_OUTAKE);
      arm.setArmTarget(AMP_ANGLE_RAD);
      

      timer.reset();
      timer.start();
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
    public void execute() {
      if(arm.armAtSetpoint() && intakeShooter.isWithinTolerance()){
        intakeShooter.setRPMIntake(EJECT_RPM_INTAKE);
        
      }

    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeShooter.stopIntake();
    intakeShooter.stopOutake();
    timer.stop();
    intakeShooter.setRumblyTumbly(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !intakeShooter.intakeDetectsNote()|| timer.hasElapsed(EJECT_TIME_SECS);
  }
}
