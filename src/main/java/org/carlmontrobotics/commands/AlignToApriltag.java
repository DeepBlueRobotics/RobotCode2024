// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.Limelight;
import static org.carlmontrobotics.Constants.Drivetrain.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import org.carlmontrobotics.subsystems.LimelightHelpers;

public class AlignToApriltag extends Command {
  //private boolean finished;
  private final Drivetrain drivetrain;
  public final TeleopDrive teleopDrive;
  public final Limelight limelight;
  //private final Pose2d target;
  private Rotation2d angle;
  public final PIDController rotationPID = new PIDController(thetaPIDController[0], thetaPIDController[1], thetaPIDController[2]);

  public AlignToApriltag(Drivetrain drivetrain, Limelight limelight) {
    addRequirements(this.drivetrain = drivetrain);
    addRequirements(this.limelight = limelight);
    //addRequirements(this.target = target);
    this.teleopDrive = (TeleopDrive) drivetrain.getDefaultCommand();
    rotationPID.enableContinuousInput(-180, 180);
    rotationPID.setSetpoint(MathUtil.inputModulus(limelight.calcAngleOffset(), -180, 180));
    rotationPID.setTolerance(positionTolerance[2], velocityTolerance[2]);
    SendableRegistry.addChild(this, rotationPID);
  }

  @Override
  public void initialize() {
    //angle = Rotation2d.fromDegrees(drivetrain.getHeading()).minus(Rotation2d.fromDegrees(limelight.calcAngleOffset()));
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight")){
      if (teleopDrive == null) drivetrain.drive(0, 0, rotationPID.calculate(drivetrain.getHeading()));
      else {
        double[] driverRequestedSpeeds = teleopDrive.getRequestedSpeeds();
        drivetrain.drive(driverRequestedSpeeds[0], driverRequestedSpeeds[1], rotationPID.calculate(drivetrain.getHeading()));
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    //return limelight.isAligned(limelight.calcAngleOffset());
    SmartDashboard.putBoolean("aligned to apriltag", rotationPID.atSetpoint());
    SmartDashboard.putNumber("apriltag align error", rotationPID.getPositionError());
    return rotationPID.atSetpoint();
  }
}