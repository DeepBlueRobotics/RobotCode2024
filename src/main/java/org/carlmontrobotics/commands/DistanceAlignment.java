package org.carlmontrobotics.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class DistanceAlignment extends Command {
  //create limelight and drivetrain instances

  public DistanceAlignment() {
    // set limelight and drivetrain as requirements
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    /* calls getTargetValid() from limelight
     * if there is a target: 
     *   -call calcDistanceAlignment from limelight
     *   -drive to adjust (function in drivetrain?):
     *      -convert offset to a number useable by drive
     *      -find out what direction drivetrain is facing
     *      -input offset values to make drivetrain move straight towards the target
     * else if already alignment:
     *   -put 0 as the dAdjustment
     *   -in other commands, use this 0 value from smartdashboard to confirm alignment before running
     * else (no target):
     *   -do nothing
     *   -put -1 as the adjustment to smartdashboard
     */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //tell drivetrain to stop
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; //placeholder
    //finsihed when the adjustment is 0
  }
}
