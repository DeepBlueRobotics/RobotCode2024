package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.Limelightc.*;
import static org.carlmontrobotics.Constants.Limelightc.Apriltag.*;


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
    Pose2d estimatedPos = poseEstimator.getEstimatedPosition();
    System.out.println(estimatedPos.getX());
    System.out.println(estimatedPos.getY());
    System.out.println(estimatedPos.getRotation().getDegrees() + "degrees");
    return poseEstimator.getEstimatedPosition();
  }


  public double getDistanceToTargetSpeaker(){
    if (LimelightHelpers.getFiducialID(SHOOTER_LL_NAME) == RED_SPEAKER_CENTER_TAG_ID || LimelightHelpers.getFiducialID(SHOOTER_LL_NAME) == BLUE_SPEAKER_CENTER_TAG_ID){
      Rotation2d angleToGoal = Rotation2d.fromDegrees(MOUNT_ANGLE_DEG_SHOOTER).plus(Rotation2d.fromDegrees(LimelightHelpers.getTY(SHOOTER_LL_NAME)));
      double distance = (SPEAKER_CENTER_HEIGHT_METERS - HEIGHT_FROM_GROUND_METERS_SHOOTER) / angleToGoal.getTan();
      SmartDashboard.putNumber("limelight distance", distance);
      return distance;
    }

    else{
      SmartDashboard.putNumber("limelight distance", -1);
      return -1;
    }
  }

  public double getDistanceToNote(){
    Rotation2d angleToGoal = Rotation2d.fromDegrees(MOUNT_ANGLE_DEG_INTAKE).plus(Rotation2d.fromDegrees(LimelightHelpers.getTY(INTAKE_LL_NAME)));
    double distance = (NOTE_HEIGHT - HEIGHT_FROM_GROUND_METERS_INTAKE) / angleToGoal.getTan();
    SmartDashboard.putNumber("limelight distance to note", distance);
    return distance;
  }

  // public Pose3d getTargetPose() {
  //   double[] poseArray = LimelightHelpers.getLimelightNTDoubleArray(SHOOTER_LL_NAME, "targetpose_robotspace");
  //   return LimelightHelpers.toPose3D(poseArray);
  // }
}