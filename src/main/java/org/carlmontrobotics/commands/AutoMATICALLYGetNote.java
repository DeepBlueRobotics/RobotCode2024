// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Limelightc.*;

import org.carlmontrobotics.Constants.Limelightc;

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
  // private Timer timer = new Timer();
  Timer timer = new Timer();
  private IntakeShooter intake;
  private int direction;

  public AutoMATICALLYGetNote(Drivetrain dt, Limelight ll,
      IntakeShooter intake, int direction) {
    addRequirements(this.dt = dt);
    this.ll = ll;
    this.intake = intake;
    this.direction = direction; // direction to turn if it doesn't see note
    //addRequirements(this.effector = effector);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {
    // timer.reset();
    // timer.start();
    // new Intake(intake).finallyDo(()->{this.end(false);});
    dt.setFieldOriented(false);
    SmartDashboard.putNumber("turning speed multiplier", 3);
    
  }


  @Override
  public void execute() {
    if (!LimelightHelpers.getTV(Limelightc.INTAKE_LL_NAME)) {
      dt.drive(0, 0, direction);
      return;
    }
    double angleErrRad = -Units
        .degreesToRadians(LimelightHelpers.getTX(Limelightc.INTAKE_LL_NAME));
    double forwardDistErrMeters = ll.getDistanceToNoteMeters(); 
    double strafeDistErrMeters = forwardDistErrMeters * Math.tan(angleErrRad);

    forwardDistErrMeters = Math.max(
        forwardDistErrMeters
            * SmartDashboard.getNumber("forward speed multiplier", 1.5),
        MIN_MOVEMENT_METERSPSEC);

    if (LimelightHelpers.getTV(INTAKE_LL_NAME)) {
      dt.drive(forwardDistErrMeters, strafeDistErrMeters, angleErrRad
          * SmartDashboard.getNumber("turning speed multiplier", 3));
    }

    // double forwardSpeed = 0;
    // double strafeSpeed = 0;
    // double rotationalSpeed = 0;

    // forwardSpeed = Math.max(
    // forwardDistErrMeters
    // * SmartDashboard.getNumber("forward speed multiplier", 1.5),
    // MIN_MOVEMENT_METERSPSEC);

    // if (LimelightHelpers.getTV(INTAKE_LL_NAME)) {
    // if (angleErrRad >= 0) {
    // strafeSpeed = Math.max(
    // strafeDistErrMeters
    // * SmartDashboard.getNumber("strafe speed multiplier", 1.5),
    // MIN_MOVEMENT_METERSPSEC);
    // rotationalSpeed = Math.max(
    // angleErrRad
    // * SmartDashboard.getNumber("rotational speed multiplier", 2),
    // MIN_MOVEMENT_RADSPSEC);
    // }

    // else {
    // strafeSpeed = Math.min(
    // strafeDistErrMeters
    // * SmartDashboard.getNumber("strafe speed multiplier", 1.5),
    // -MIN_MOVEMENT_METERSPSEC);
    // rotationalSpeed = Math.min(
    // angleErrRad
    // * SmartDashboard.getNumber("rotational speed multiplier", 2),
    // -MIN_MOVEMENT_RADSPSEC);
    // }

    // if (angleErrRad >= -ERROR_TOLERANCE_RAD
    // && angleErrRad <= ERROR_TOLERANCE_RAD) {
    // rotationalSpeed = 0;
    // }

    // dt.drive(forwardSpeed, strafeSpeed, rotationalSpeed);


    // 180deg is about 6.2 rad/sec, min is .5rad/sec
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.setFieldOriented(true);
    intake.stopIntake();

    // SmartDashboard.putBoolean("end", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.outtakeDetectsNote();
  }
}
