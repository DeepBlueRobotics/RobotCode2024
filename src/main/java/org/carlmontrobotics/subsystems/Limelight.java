package org.carlmontrobotics.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  //public boolean targetValid = false;

  public Limelight() {}

  @Override
  public void periodic() {
    //calls tracking()
  }

  public void tracking() {
    /* gets certain limelight values (ta, tx, ty, tv, tid, etc)
    checks if there is a target and updates targetValid
    puts those values to SmartDashboard
    */
  }
  
  public boolean getTargetValid(){
    return false; //placeholder
    //returns targetValid
  }

  public double getTargetID(){
    return 0; //placeholder
    //returns the id of the detected apriltag
  }

  public double calcDistanceAlignment(){
    return 0; //placeholder
    /* -calculate adjustment
     * -put adjustment to smartdashboard
     * -return adjustment
     */
    //figure out the math to do this
  }
}
