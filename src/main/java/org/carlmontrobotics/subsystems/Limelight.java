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
    /* gets certain limelight values (ta, tx, ty, tv, etc)
    checks if there is a target and updates targetValid
    puts those values to SmartDashboard
    */
  }
  
  public void getTargetValid(){
    //returns targetValid
  }

  public void calcDistanceAlignment(){
    /* -calculate adjustment
     * -put adjustment to smartdashboard
     */
    //figure out the math to do this
  }
}
