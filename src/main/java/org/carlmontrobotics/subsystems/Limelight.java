package org.carlmontrobotics.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  //public boolean targetValid = false;
  //private double tid, tv, tx, ty, ta;
  //private double distOffset, horizOffset;
  //private double horizHeadingError, horizAdjust;

  public Limelight() {}

  @Override
  public void periodic() {
    //tracking();
  }

  public void tracking() {
    /* gets certain limelight values (ta, tx, ty, tv, tid, etc)
    checks if there is a target and updates targetValid
    puts those values to SmartDashboard
    */

    /* 
     * tid = NetworkTableInstance.getDefault().getTable(“limelight”).getEntry(“tid”).getDouble(0);
     * tv = NetworkTableInstance.getDefault().getTable(“limelight”).getEntry("tv").getDouble(0);
     * tx = NetworkTableInstance.getDefault().getTable(“limelight”).getEntry("tx").getDouble(0);
     * ty = NetworkTableInstance.getDefault().getTable(“limelight”).getEntry("ty").getDouble(0);
     * ta = NetworkTableInstance.getDefault().getTable(“limelight”).getEntry("ta").getDouble(0);
     * 
     * if (tv == 1.0){
     *   targetValid = true;
     * } else {
     *   targetValid = false;
     * }
     * 
     * SmartDashboard.putNumber("apriltag id", tid);
     * SmartDashboard.putBoolean("target in sight", targetValid);
     * SmartDashboard.putNumber("horizontal offset", tx);
     * SmartDashboard.putNumber("vertical offset", ty);
     * SmartDashboard.putNumber("target area", ta);
    */
  }
  
  public boolean getTargetValid(){
    return false;
    //return targetValid;
  }

  public double getTargetID(){
    return 0; //placeholder
    //returns the id of the detected apriltag
    //return tid;
  }

  public double calcDistanceAlignment(){
    return 0; //placeholder
    /* -calculate dAdjustment
     * -put dAdjustment to smartdashboard
     * -return dAdjustment
     */
    //figure out the math to do this
  }

  public double calcHorizontalAlignment(){
    return 0; //placeholder
    /* -calculate hAdjustment
     * -put hAdjustment to smartdashboard
     * -return hAdjustment 
    */

  }
}
