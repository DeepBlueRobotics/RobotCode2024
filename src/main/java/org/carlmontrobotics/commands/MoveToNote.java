// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import org.carlmontrobotics.Constants.Limelightc;
import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.Limelight;
import org.carlmontrobotics.subsystems.LimelightHelpers;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveToNote extends Command {
  private final Drivetrain dt;
  private final Limelight ll;
  private Timer timer = new Timer();
  private boolean originalFieldOrientation;
  /** Creates a new MoveToNote. */
  public MoveToNote(Drivetrain dt, Limelight ll) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.dt=dt);
    this.ll = ll;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    originalFieldOrientation = dt.getFieldOriented();
    timer.reset();
    timer.start();
    dt.setFieldOriented(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double radErr = Units.degreesToRadians(LimelightHelpers.getTX(Limelightc.INTAKE_LL_NAME));
    double distErr = ll.getDistanceToNoteMeters(); //meters
    double forwardErr = distErr * Math.cos(radErr);
    dt.drive(Math.max(forwardErr*2, .5), 0, 0);
    //180deg is about 6.2 rad/sec, min is .5rad/sec
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.setFieldOriented(originalFieldOrientation);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= 0.5;
  }
}
