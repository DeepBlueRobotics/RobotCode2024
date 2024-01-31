// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class HorizontalAlignment extends Command {
  //private boolean finished;
  //create limelight and drivetrain instances
  public HorizontalAlignment() {
    //set limelight and drivetrain as requirements
  }

  @Override
  public void initialize() {
    //private double distance;
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
  }

  @Override
  public void end(boolean interrupted) {
    //tell drivetrain to stop
  }

  @Override
  public boolean isFinished() {
    return false;
    //return finished;
  }
}
