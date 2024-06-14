package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Armc.SMART_CURRENT_LIMIT_TIMEOUT;
import static org.carlmontrobotics.Constants.Effectorc.*;
import org.carlmontrobotics.subsystems.Arm;
import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonRuinerShoot extends Command {
  private IntakeShooter intakeShooter;
  private Timer timer = new Timer();

  public AutonRuinerShoot(IntakeShooter intakeShooter) {
    this.intakeShooter = intakeShooter;
    addRequirements(intakeShooter);
    timer.stop();
    timer.reset();
  }

  @Override
  public void initialize() {
    intakeShooter.setMaxOuttake(0.5);
    timer.start();

  }

  @Override
  public void execute() {
  }


  @Override
  public void end(boolean interrupted) {
    intakeShooter.stopOuttake();
  }

  @Override
  public boolean isFinished() {
    return timer.get() > 15;
  }
}
