// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveForwardRobotOriented extends Command {
  private final Drivetrain dt;
  /** Creates a new DriveForwardRobotOriented. */
  public DriveForwardRobotOriented(Drivetrain dt) {
    addRequirements(this.dt = dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.setFieldOriented(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dt.drive(0.5, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.setFieldOriented(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
