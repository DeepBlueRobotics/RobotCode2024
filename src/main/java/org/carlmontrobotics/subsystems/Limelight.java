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
  //private final Drivetrain drivetrain;
  //private final SwerveDrivePoseEstimator poseEstimator;
  private double tv, tx;
  private double[] botPose = null;
  private double[] targetPose = null;
  private Pose3d botpose;

  //private double distOffset, horizOffset;
  //private double horizHeadingError, horizAdjust;

  public Limelight() {
    // this.drivetrain = drivetrain;
    // poseEstimator = new SwerveDrivePoseEstimator(
    //   drivetrain.getKinematics(), 
    //   Rotation2d.fromDegrees(drivetrain.getHeading()), 
    //   drivetrain.getModulePositions(), 
    //   new Pose2d());
    
    botPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[7]);
    //targetPose = table.getEntry("targetpose_fieldspace").getDoubleArray(new double[7]);
  }

  @Override
  public void periodic() {
    distanceToTargetSpeaker();
    //getCurrentPose();
  }


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


  /*
  constants:


  functions:
  getBotpose3d vs getCurrentPose

  */
}