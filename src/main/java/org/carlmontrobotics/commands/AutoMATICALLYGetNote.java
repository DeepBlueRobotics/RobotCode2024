// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import org.carlmontrobotics.Constants.*;
import org.carlmontrobotics.subsystems.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  //private IntakeShooter effector;
  private Limelight ll;
  private Timer timer = new Timer();

  public AutoMATICALLYGetNote(Drivetrain dt, Limelight ll /*IntakeShooter effector*/) {
    addRequirements(this.dt = dt);
    addRequirements(this.ll = ll);
    //addRequirements(this.effector = effector);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    //new Intake().finallyDo(()->{this.end(false);});
    SmartDashboard.putBoolean("end", false);
    dt.setFieldOriented(false);
  }

  @Override
  public void execute() {
    double radErr = Units.degreesToRadians(LimelightHelpers.getTX(Limelightc.INTAKE_LL_NAME));
    double distErr = ll.getDistanceToNote(); //meters
    double forwardErr = distErr * Math.cos(radErr);
    double strafeErr = distErr * Math.sin(radErr);
    // dt.drive(0,0,0);
    dt.drive(Math.max(forwardErr*2, .5), Math.max(strafeErr*2, .5), Math.max(radErr*2,.5));
    //180deg is about 6.2 rad/sec, min is .5rad/sec
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.setFieldOriented(true);
    SmartDashboard.putBoolean("end", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //return timer.get() >= 0.5;
  }
}
