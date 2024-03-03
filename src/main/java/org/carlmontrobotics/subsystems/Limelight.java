package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.Limelight.*;
import static org.carlmontrobotics.Constants.Limelight.Apriltag.*;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
    getDistanceToTargetSpeaker();
    getCurrentPose();
  }

  public void updateBotPose3d(){
    botpose = LimelightHelpers.getBotPose3d("limelight");
  }


  public Pose2d getCurrentPose(){
    System.out.println(poseEstimator.getEstimatedPosition());
    return poseEstimator.getEstimatedPosition();
  }


  public double getDistanceToTargetSpeaker(){
    if (LimelightHelpers.getFiducialID("limelight") == SPEAKER_CENTER_TAG_ID_1 || LimelightHelpers.getFiducialID("limelight") == SPEAKER_CENTER_TAG_ID_2){
      double angleToGoalRadians = Units.degreesToRadians(MOUNT_ANGLE_DEG + LimelightHelpers.getTY("limelight"));
      double distance = (SPEAKER_CENTER_HEIGHT_METERS - HEIGHT_FROM_GROUND_METERS) / Math.tan(angleToGoalRadians);
      SmartDashboard.putNumber("limelight distance", distance);
      return distance;
    }
    else{
      SmartDashboard.putNumber("limelight distance", -1);
      return -1;
    }
  }

  // public Pose3d getTargetPose() {
  //   double[] poseArray = LimelightHelpers.getLimelightNTDoubleArray("limelight", "targetpose_robotspace");
  //   return LimelightHelpers.toPose3D(poseArray);
  // }

  // public double distanceToTargetmath(Pose3d target){
  //   return(6.5 inches)/2 x tan(2 * 320 pixels)
  //   return (tag width in real world)/(2 x tan((tag pixel width/(2 * horizontal resolution)) * pi/180));
  // }
}