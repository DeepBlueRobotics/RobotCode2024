package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.Limelightc.*;
import static org.carlmontrobotics.Constants.Limelightc.Apriltag.*;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private final Drivetrain drivetrain;
  private final SwerveDrivePoseEstimator poseEstimator;
  // private double tv, tx;
  // private double[] rawBotPose = null;
  // private double[] targetPose = null;
  private Pose3d botPose;

  public Limelight(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    poseEstimator = new SwerveDrivePoseEstimator(
        drivetrain.getKinematics(),
        Rotation2d.fromDegrees(drivetrain.getHeading()),
        drivetrain.getModulePositions(),
        new Pose2d());
  }

  @Override
  public void periodic() {
    poseEstimator.update(Rotation2d.fromDegrees(drivetrain.getHeading()), drivetrain.getModulePositions());
    updateBotPose3d();
    getDistanceToSpeakerMeters();
    getCurrentPose();
    getDistanceToNoteMeters();
  }


  public void updateBotPose3d() {
    botPose = LimelightHelpers.getBotPose3d(SHOOTER_LL_NAME);
  }

  public Pose2d getCurrentPose() {
    Pose2d estimatedPos = poseEstimator.getEstimatedPosition();
    SmartDashboard.putNumber("estimated x", estimatedPos.getX());
    // SmartDashboard.putNumber("estimated y", estimatedPos.getY());
    // SmartDashboard.putNumber("estimated rotation (deg)", estimatedPos.getRotation().getDegrees());
    return estimatedPos;
  }


  public double getTXDeg(String limelightName) {
    return (limelightName == INTAKE_LL_NAME) ? LimelightHelpers.getTX(INTAKE_LL_NAME) : -LimelightHelpers.getTY(SHOOTER_LL_NAME);
  }

  public double getTYDeg(String limelightName) {
    return (limelightName == INTAKE_LL_NAME) ? LimelightHelpers.getTY(INTAKE_LL_NAME) : LimelightHelpers.getTX(SHOOTER_LL_NAME);
  }

  public double getDistanceToSpeakerMeters() {
    if (LimelightHelpers.getFiducialID(SHOOTER_LL_NAME) == RED_SPEAKER_CENTER_TAG_ID
        || LimelightHelpers.getFiducialID(SHOOTER_LL_NAME) == BLUE_SPEAKER_CENTER_TAG_ID) {
          // TODO: change MOUNT_ANGLE_DEG_SHOOTER
      Rotation2d angleToGoal = Rotation2d.fromDegrees(MOUNT_ANGLE_DEG_SHOOTER)
          .plus(Rotation2d.fromDegrees(getTYDeg(SHOOTER_LL_NAME))); //because limelight is mounted horizontally
      double distance = (SPEAKER_CENTER_HEIGHT_METERS - HEIGHT_FROM_GROUND_METERS_SHOOTER) / angleToGoal.getTan();
      // SmartDashboard.putNumber("limelight distance", distance);
      return distance;
    }

    else {
      // SmartDashboard.putNumber("limelight distance", -1);
      return -1;
    }
  }


  public double getDistanceToNoteMeters() {
    Rotation2d angleToGoal = Rotation2d.fromDegrees(MOUNT_ANGLE_DEG_INTAKE)
        .plus(Rotation2d.fromDegrees(getTYDeg(INTAKE_LL_NAME)));
    if (angleToGoal.getDegrees() <= 0) {
      double distance = (HEIGHT_FROM_GROUND_METERS_INTAKE - NOTE_HEIGHT) / Math.tan(Math.abs(angleToGoal.getRadians()));
      // SmartDashboard.putNumber("limelight distance", distance);
      return distance;
    } else {
      // SmartDashboard.putNumber("limelight distance", -1);
      return -1;
    }
  }

  public double getArmAngleToShootSpeakerRad(){
    double armRestingHeightToSubwooferMeters = HEIGHT_FROM_RESTING_ARM_TO_SPEAKER_METERS;
    double horizontalDistanceMeters = getDistanceToSpeakerMeters() + SIDEWAYS_OFFSET_TO_OUTTAKE_MOUTH;
    return END_EFFECTOR_BASE_ANGLE_RADS - Math.atan(armRestingHeightToSubwooferMeters / horizontalDistanceMeters);
  }

  public double getRotateAngleDeg() {
    double cameraLensHorizontalOffset = getTXDeg(SHOOTER_LL_NAME) / getDistanceToSpeakerMeters();
    double realHorizontalOffset = Math.atan(cameraLensHorizontalOffset / getDistanceToSpeakerMeters());
    return Math.atan(realHorizontalOffset / getDistanceToSpeakerMeters());
  }

}