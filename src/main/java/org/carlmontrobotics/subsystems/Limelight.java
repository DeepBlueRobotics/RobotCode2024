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
  private Pose3d botpose;

  //private double distOffset, horizOffset;
  //private double horizHeadingError, horizAdjust;

  public Limelight(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    poseEstimator = new SwerveDrivePoseEstimator(
      drivetrain.getKinematics(), 
      Rotation2d.fromDegrees(drivetrain.getHeading()), 
      drivetrain.getModulePositions(), 
      new Pose2d());
    
    botPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[7]);
    //targetPose = table.getEntry("targetpose_fieldspace").getDoubleArray(new double[7]);
  }

  @Override
  public void periodic() {
    distanceToTargetSpeaker();
  }

  public double calcAngleOffset(){
    //-calculates the angle to turn in degrees
    //if it goes wrong uncomment
    return LimelightHelpers.getTX("limelight"); //* (Constants.Limelight.horizontalFOV / Constants.Limelight.resolutionWidth);
  }

  public void updateBotPose3d(){
    botpose = LimelightHelpers.getBotPose3d("limelight");
  }

  public boolean isAligned(double distance){
    return MathUtil.applyDeadband(distance, Constants.Limelight.errorTolerance) == 0; //if it's within 0.1 of the center
  }

  public Pose2d getCurrentPose(){
    return poseEstimator.getEstimatedPosition();
  }

  // public Pose3d getTargetPose() {
  //   double[] poseArray = LimelightHelpers.getLimelightNTDoubleArray("limelight", "targetpose_robotspace");
  //   return LimelightHelpers.toPose3D(poseArray);
  // }

  // public double distanceToTargetmath(Pose3d target){
  //   return(6.5 inches)/2 x tan(2 * 320 pixels)
  //   return (tag width in real world)/(2 x tan((tag pixel width/(2 * horizontal resolution)) * pi/180));
  // }

  public double distanceToTargetSpeaker(){
    if (LimelightHelpers.getFiducialID("limelight") == 4 || LimelightHelpers.getFiducialID("limelight") == 7){
      double angleToGoalRadians = (Constants.Limelight.mountAngleDeg + LimelightHelpers.getTY("limelight")) * (Math.PI/180);
      double distance = (Constants.Limelight.Apriltag.speakerCenterHeightMeters - Constants.Limelight.heightFromGroundMeters) / Math.tan(angleToGoalRadians);
      SmartDashboard.putNumber("limelight distance", distance);
      return distance;
    }
    else{
      SmartDashboard.putNumber("limelight distance", -1);
      return -1;
    }
  }

    // public double distanceToTargetxyz(){
  //   if (LimelightHelpers.getFiducialID("limelight") == 4 || LimelightHelpers.getFiducialID("limelight") == 7){
  //     Pose3d target = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
  //     return Math.sqrt(target.getX() * target.getX() + target.getY() * target.getY() + target.getZ() * target.getZ());
  //   }
  //   else{
  //     return 0;
  //   }
  // }

  /*TODO
  constants:
  horizontalFOV

  functions:
  test distanceToTargetxyz and distanceToTargetSpeakerCenter
  getBotpose3d vs getCurrentPose

  */
}