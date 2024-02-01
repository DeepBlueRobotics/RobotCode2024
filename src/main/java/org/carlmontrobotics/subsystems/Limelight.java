package org.carlmontrobotics.subsystems;

import org.carlmontrobotics.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private double tid, tv, tx, ty, ta;
  //private double distOffset, horizOffset;
  //private double horizHeadingError, horizAdjust;

  public Limelight() {}

  @Override
  public void periodic() {
    tracking();
  }

  public void tracking() {
    tid = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    /* gets certain limelight values (ta, tx, ty, tv, tid, etc)
    checks if there is a target and updates targetValid
    puts those values to SmartDashboard
    */
    
     /* 
     * SmartDashboard.putNumber("apriltag id", tid);
     * SmartDashboard.putBoolean("target in sight", targetValid);
     * SmartDashboard.putNumber("horizontal offset", tx);
     * SmartDashboard.putNumber("vertical offset", ty);
     * SmartDashboard.putNumber("target area", ta);
    */
  }
  
  public boolean getTargetValid(){
    return tv == 1.0;
  }

  public double getTargetID(){
    //returns the id of the detected apriltag
    return tid;
  }

  public double calcAngleOffset(){
    return 0; //placeholder
    //-calculates the angle to turn in degrees
  }

  public boolean isAligned(double distance){
    return MathUtil.applyDeadband(distance, Constants.Limelight.errorTolerance) == 0; //if it's within 0.1 of the center
  }
}