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
  //private double[] rawBotPose = null;
  private double[] targetPose = null;
  private Pose3d botPose;

  //private double distOffset, horizOffset;
  //private double horizHeadingError, horizAdjust;

  public Limelight(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    poseEstimator = new SwerveDrivePoseEstimator(
      drivetrain.getKinematics(), 
      Rotation2d.fromDegrees(drivetrain.getHeading()), 
      drivetrain.getModulePositions(), 
      new Pose2d());
    
    //rawBotPose = NetworkTableInstance.getDefault().getTable(SHOOTER_LL_NAME).getEntry("botpose").getDoubleArray(new double[7]);
    //targetPose = table.getEntry("targetpose_fieldspace").getDoubleArray(new double[7]);
  }

  @Override
  public void periodic() {
    poseEstimator.update(Rotation2d.fromDegrees(drivetrain.getHeading()), drivetrain.getModulePositions());
    updateBotPose3d();
    getDistanceToTargetSpeaker();
    getCurrentPose();
  }

  public void updateBotPose3d(){
    botPose = LimelightHelpers.getBotPose3d(SHOOTER_LL_NAME);
  }


  public Pose2d getCurrentPose(){
    System.out.println(poseEstimator.getEstimatedPosition());
    return poseEstimator.getEstimatedPosition();
  }


  public double getDistanceToTargetSpeaker(){
    if (LimelightHelpers.getFiducialID(SHOOTER_LL_NAME) == SPEAKER_CENTER_TAG_ID_LEFT || LimelightHelpers.getFiducialID(SHOOTER_LL_NAME) == SPEAKER_CENTER_TAG_ID_RIGHT){
      Rotation2d angleToGoal = Rotation2d.fromDegrees(MOUNT_ANGLE_DEG).plus(Rotation2d.fromDegrees(LimelightHelpers.getTY(SHOOTER_LL_NAME)));
      double distance = (SPEAKER_CENTER_HEIGHT_METERS - HEIGHT_FROM_GROUND_METERS) / angleToGoal.getTan();
      SmartDashboard.putNumber("limelight distance", distance);
      return distance;
    }

    else{
      SmartDashboard.putNumber("limelight distance", -1);
      return -1;
    }
  }

  // public Pose3d getTargetPose() {
  //   double[] poseArray = LimelightHelpers.getLimelightNTDoubleArray(SHOOTER_LL_NAME, "targetpose_robotspace");
  //   return LimelightHelpers.toPose3D(poseArray);
  // }
}