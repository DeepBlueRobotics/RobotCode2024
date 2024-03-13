package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Effectorc.*;
import static org.carlmontrobotics.Constants.Led.*;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.AuxSystems;
import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
public class RampToRPM extends Command {
    //intake until sees game peice or 4sec has passed
    private final double rpm;
    private final IntakeShooter intake;
    private Timer timer;

    public RampToRPM(IntakeShooter intake, double rpm) {
        addRequirements(this.intake = intake);
        this.rpm=rpm;
    }

    @Override
    public void initialize() {
      intake.setRPMOutake(rpm);
      timer.reset();
      timer.start();
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.resetCurrentLimit();
    timer.stop();
    //resets to defaultColor
  }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return intake.isWithinTolerance() || timer.hasElapsed(1.5);
    }
}
