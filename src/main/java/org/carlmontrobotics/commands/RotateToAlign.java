// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.Limelight;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateToAlign extends Command {
  private boolean finished;
  Drivetrain drivetrain;
  Limelight limelight;

  public RotateToAlign(Drivetrain drivetrain, Limelight limelight) {
    addRequirements(this.drivetrain = drivetrain);
    addRequirements(this.limelight = limelight);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    /* calls getTargetValid() from limelight
     * if there is a target: 
     *   -call calcHorizontalAlignment from limelight
     *   -rotate by calling the command RotateToFieldRelativeAngle
     * else if there is a targt and limelight.isAligned is true:
     *   -put 0 as the adjustment
     *   -set finished to true
     * else (no target):
     *   -do nothing
     *   -put -1 as the adjustment to smartdashboard
     *   -set finished to true
     */
    if (limelight.getTargetValid()){
      //make a sequential command group that makes it turn then shoot
      //new SequentialCommandGroup(new Eject(outtake), new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(limelight.calcAngleOffset()), drivetrain);)
      new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(limelight.calcAngleOffset()), drivetrain);
    } else{
      finished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    //tell drivetrain to stop
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}
