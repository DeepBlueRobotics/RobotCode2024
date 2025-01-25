// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class OppositeEject extends Command {
  private final IntakeShooter intake;
  private final Timer timer = new Timer();

  /** Creates a new IntactEject. */
  public OppositeEject(IntakeShooter intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intake = intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    intake.setMaxIntake(-1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    timer.stop();
    intake.resetCurrentLimit();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.hasElapsed(1.5));
  }
}
