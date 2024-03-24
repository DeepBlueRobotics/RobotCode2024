// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Limelightc.thetaPControl;
import static org.carlmontrobotics.Constants.Limelightc.xyPControl;

import org.carlmontrobotics.Constants.*;
import org.carlmontrobotics.subsystems.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoMATICALLYGetNote extends Command {
  /** Creates a new AutoMATICALLYGetNote. */
  private Drivetrain dt;
  private IntakeShooter effector;
  private Limelight limelight;

  public AutoMATICALLYGetNote(Drivetrain dt /*IntakeShooter effector*/) {
    addRequirements(this.dt = dt);
    //addRequirements(this.effector = effector);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {
    //new Intake().finallyDo(()->{this.end(false);});
    dt.setFieldOriented(false);
  }

  @Override
  public void execute() {
    double angleErrRad = Units.degreesToRadians(LimelightHelpers.getTX(Limelightc.INTAKE_LL_NAME));
    double distErrMeters = limelight.getDistanceToNote(); //meters
    double forwardErrMeters = distErrMeters * Math.cos(angleErrRad);
    double strafeErrMeters = distErrMeters * Math.sin(angleErrRad);
    dt.drive(Math.max(forwardErrMeters*xyPControl, .5), Math.max(strafeErrMeters*xyPControl, .5), Math.max(angleErrRad*thetaPControl,.5));
    //180deg is about 6.2 rad/sec, min is .5rad/sec
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.setFieldOriented(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
