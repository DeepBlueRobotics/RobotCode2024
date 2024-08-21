package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Armc.GROUND_INTAKE_POS;
import static org.carlmontrobotics.Constants.Armc.SMART_CURRENT_LIMIT_TIMEOUT;
import static org.carlmontrobotics.Constants.Effectorc.*;
import org.carlmontrobotics.subsystems.Arm;
import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonRuinerShootAndIntake extends Command {
  private IntakeShooter intakeShooter;
  private Arm arm;

  public AutonRuinerShootAndIntake(IntakeShooter intakeShooter, Arm arm) {
    this.intakeShooter = intakeShooter;
    this.arm = arm;
    addRequirements(intakeShooter);
  }

  @Override
  public void initialize() {
    new ArmToPos(arm, GROUND_INTAKE_POS - 0.02);
    intakeShooter.setRPMIntake(INTAKE_RPM);
    intakeShooter.motorSetOutake(0.3);

  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    intakeShooter.stopOuttake();
    intakeShooter.stopIntake();

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
