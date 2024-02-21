package org.carlmontrobotics.subsystems;

import org.carlmontrobotics.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private final Drivetrain drivetrain;
  private final SwerveDrivePoseEstimator poseEstimator;
  private double tv, tx;
  private double[] botPose = null;
  private double[] targetPose = null;

  //private double distOffset, horizOffset;
  //private double horizHeadingError, horizAdjust;

  public Limelight(Drivetrain drivetrain) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    this.drivetrain = drivetrain;
    poseEstimator = new SwerveDrivePoseEstimator(
      drivetrain.getKinematics(), 
      Rotation2d.fromDegrees(drivetrain.getHeading()), 
      drivetrain.getModulePositions(), 
      new Pose2d());
    
    botPose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
    targetPose = table.getEntry("targetpose_robotspace").getDoubleArray(new double[7]);
  }

  @Override
  public void periodic() {
    //tracking();
    SmartDashboard.putNumber("angle offset", calcAngleOffset());
    // for(double value:botPose){
    //   System.out.println(value);
    // }
    for(int i=0; i<7; i++){
      System.out.println(botPose[i]);
    }
    // for(double value:targetPose){
    //   System.out.println(value);
    // }
    // System.out.println(LimelightHelpers.getBotPose("limelight"));
    // System.out.println(LimelightHelpers.getTargetPose_RobotSpace("limelight"));
  }

  public double calcAngleOffset(){
    //-calculates the angle to turn in degrees
    return getTx() * (Constants.Limelight.horizontalFOV / Constants.Limelight.resolutionWidth);
  }

  public boolean isAligned(double distance){
    return MathUtil.applyDeadband(distance, Constants.Limelight.errorTolerance) == 0; //if it's within 0.1 of the center
  }

  public Pose2d getCurrentPose(){
    return poseEstimator.getEstimatedPosition();
  }

  public double getTv(){
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    return tv;
  }

  public double getTx(){
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    return tx;
  }

  public Pose2d getTargetPose() {
    double[] poseArray = LimelightHelpers.getLimelightNTDoubleArray("limelight", "targetpose_robotspace");
    return LimelightHelpers.toPose2D(poseArray);
  }

  // public double getDistanceApriltag(){
  //   return ()/(2*Math.tan((ta)/(2*)));
  // }
}