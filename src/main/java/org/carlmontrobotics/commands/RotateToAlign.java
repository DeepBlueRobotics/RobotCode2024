// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.Limelight;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import org.carlmontrobotics.subsystems.LimelightHelpers;

public class RotateToAlign extends Command {
  //private boolean finished;
  private final Drivetrain drivetrain;
  private final Limelight limelight;
  //private final Pose2d target;
  private Rotation2d angle;

  public RotateToAlign(Drivetrain drivetrain, Limelight limelight) {
    addRequirements(this.drivetrain = drivetrain);
    addRequirements(this.limelight = limelight);
    //addRequirements(this.target = target);
  }

  @Override
  public void initialize() {
    angle = Rotation2d.fromDegrees(drivetrain.getHeading()).minus(Rotation2d.fromDegrees(limelight.calcAngleOffset()));
  }

  @Override
  public void execute() {
    if (limelight.getTv() == 1.0){
      //new SequentialCommandGroup(new Eject(outtake), new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(limelight.calcAngleOffset()), drivetrain);)
      new RotateToFieldRelativeAngle(angle, drivetrain);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return limelight.isAligned(limelight.calcAngleOffset());
  }
}
