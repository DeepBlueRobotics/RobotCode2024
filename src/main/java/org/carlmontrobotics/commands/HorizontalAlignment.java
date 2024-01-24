// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class HorizontalAlignment extends Command {
  //create limelight and drivetrain instances

  public HorizontalAlignment() {
    //set limelight and drivetrain as requirements
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    /* calls getTargetValid() from limelight
     * if there is a target: 
     *   -call calcHorizontalAlignment from limelight
     *   -drive to adjust (function in drivetrain?):
     *      -convert offset to a number useable by drive
     *      -find out what direction drivetrain is facing
     *      -input offset values to make drivetrain move left/right towards the target
     * else if already alignment:
     *   -put 0 as the adjustment
     *   -in other commands, use this 0 value from smartdashboard to confirm alignment before running
     * else (no target):
     *   -do nothing
     *   -put -1 as the adjustment to smartdashboard
     */
  }

  @Override
  public void end(boolean interrupted) {
    //tell drivetrain to stop
  }

  @Override
  public boolean isFinished() {
    return false;
    //when the offset is 0
  }
}
