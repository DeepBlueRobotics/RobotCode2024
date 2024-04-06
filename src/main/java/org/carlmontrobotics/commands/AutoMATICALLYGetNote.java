// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Limelightc.*;
import static org.carlmontrobotics.Constants.Effectorc.*;

import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.IntakeShooter;
import org.carlmontrobotics.subsystems.Limelight;
import org.carlmontrobotics.subsystems.LimelightHelpers;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoMATICALLYGetNote extends Command {
  /** Creates a new AutoMATICALLYGetNote. */
  private Drivetrain dt;
  // private IntakeShooter effector;
  private Limelight ll;
  private IntakeShooter intake;
  //private Timer timer = new Timer();
  private int index;
  private int increaseAmount = 250;
  public AutoMATICALLYGetNote(Drivetrain dt, IntakeShooter intake, Limelight ll) {
    addRequirements(this.dt = dt);
    addRequirements(this.intake = intake);
    this.ll = ll;
    //addRequirements(this.effector = effector);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {
    // timer.reset();
    // timer.start();
    // new Intake(intake).finallyDo(()->{this.end(false);});
    dt.setFieldOriented(false);
    intake.motorSetIntake(0.5);
    
  }

  @Override
  public void execute() {
    double angleErrRad = Units.degreesToRadians(LimelightHelpers.getTX(INTAKE_LL_NAME));
    double forwardDistErrMeters = ll.getDistanceToNoteMeters();
    double strafeDistErrMeters = forwardDistErrMeters * Math.tan(angleErrRad);
    // dt.drive(0,0,0);
    dt.drive(Math.max(forwardDistErrMeters * 2, MIN_MOVEMENT_METERSPSEC),
        Math.max(strafeDistErrMeters * 2, MIN_MOVEMENT_METERSPSEC), Math.max(angleErrRad * 2, MIN_MOVEMENT_RADSPSEC));
    // 180deg is about 6.2 rad/sec, min is .5rad/sec

    if (LimelightHelpers.getTV(INTAKE_LL_NAME)) {
      intake.motorSetIntake(0.5);
    }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.setFieldOriented(true);
    //SmartDashboard.putBoolean("end", true);
    intake.stopIntake();
   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (intake.intakeDetectsNote());
    //return timer.get() >= 0.5;
  }
}
