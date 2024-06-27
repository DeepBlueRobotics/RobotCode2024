package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.Limelightc.*;
import static org.carlmontrobotics.Constants.Limelightc.Apriltag.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private final Drivetrain drivetrain;
  // private final SwerveDrivePoseEstimator poseEstimator;

  // private double tv, tx;
  // private double[] rawBotPose = null;
  // private double[] targetPose = null;

  private final InterpolatingDoubleTreeMap shooterMap;

  public Limelight(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    // poseEstimator = new SwerveDrivePoseEstimator(
    // drivetrain.getKinematics(),
    // Rotation2d.fromDegrees(drivetrain.getHeading()),
    // drivetrain.getModulePositions(),
    // new Pose2d());

    LimelightHelpers.SetFiducialIDFiltersOverride(SHOOTER_LL_NAME, VALID_IDS);

    shooterMap = new InterpolatingDoubleTreeMap(); // add values after testing
    // key is distance (meters), value is angle (rads)
    shooterMap.put(0.0, 0.0);
  }

  @Override
  public void periodic() {
    // poseEstimator.update(Rotation2d.fromDegrees(drivetrain.getHeading()),
    // drivetrain.getModulePositions());

    // updateMT2Odometry();

    // intake limelight testing
    SmartDashboard.putBoolean("see note", LimelightHelpers.getTV(INTAKE_LL_NAME));
    SmartDashboard.putNumber("distance to note", getDistanceToNoteMeters());
    SmartDashboard.putNumber("intake tx", LimelightHelpers.getTX(INTAKE_LL_NAME));
    SmartDashboard.putNumber("rotation to align", getRotateAngleRad());

    // shooter limelight testing
    SmartDashboard.putNumber("distance to speaker (meters)", getDistanceToSpeakerMetersMT2());
    SmartDashboard.putNumber("optimized arm angle", getArmAngleToShootSpeakerRad());
  }

  // public Pose2d getCurrentPose() {
  // Pose2d estimatedPos = poseEstimator.getEstimatedPosition();
  // SmartDashboard.putNumber("estimated x", estimatedPos.getX());
  // // SmartDashboard.putNumber("estimated y", estimatedPos.getY());
  // // SmartDashboard.putNumber("estimated rotation (deg)",
  // estimatedPos.getRotation().getDegrees());
  // return estimatedPos;
  // }

  public double getTXDeg(String limelightName) {
    return (limelightName == INTAKE_LL_NAME) ? LimelightHelpers.getTX(INTAKE_LL_NAME) : -LimelightHelpers.getTY(SHOOTER_LL_NAME);
  }

  public double getTYDeg(String limelightName) {
    return (limelightName == INTAKE_LL_NAME) ? LimelightHelpers.getTY(INTAKE_LL_NAME) : LimelightHelpers.getTX(SHOOTER_LL_NAME);
  }

  public double getDistanceToSpeakerMeters() {
    if (LimelightHelpers.getFiducialID(SHOOTER_LL_NAME) == RED_SPEAKER_CENTER_TAG_ID
        || LimelightHelpers.getFiducialID(SHOOTER_LL_NAME) == BLUE_SPEAKER_CENTER_TAG_ID) {
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

  public double getArmAngleToShootSpeakerRad() {
    double armRestingHeightToSubwooferMeters = HEIGHT_FROM_RESTING_ARM_TO_SPEAKER_METERS;
    double horizontalDistanceMeters = getDistanceToSpeakerMeters() + SIDEWAYS_OFFSET_TO_OUTTAKE_MOUTH;
    return END_EFFECTOR_BASE_ANGLE_RADS - Math.atan(armRestingHeightToSubwooferMeters / horizontalDistanceMeters);
  }

  public double getRotateAngleRad() {
    double cameraLensHorizontalOffset = getTXDeg(SHOOTER_LL_NAME) / getDistanceToSpeakerMeters();
    double realHorizontalOffset = Math.atan(cameraLensHorizontalOffset / getDistanceToSpeakerMeters());
    return Math.atan(realHorizontalOffset / getDistanceToSpeakerMeters());
  }

  // megatag2

  // public void updateMT2Odometry() {
  // boolean rejectVisionUpdate = false;

  // LimelightHelpers.SetRobotOrientation(SHOOTER_LL_NAME,
  // poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0,
  // 0);
  // LimelightHelpers.PoseEstimate visionPoseEstimate = LimelightHelpers
  // .getBotPoseEstimate_wpiBlue_MegaTag2(SHOOTER_LL_NAME);

  // if (Math.abs(drivetrain.getGyroRate()) > MAX_TRUSTED_ANG_VEL_DEG_PER_SEC) {
  // // degrees per second
  // rejectVisionUpdate = true;
  // }

  // if (visionPoseEstimate.tagCount == 0) {
  // rejectVisionUpdate = true;
  // }

  // if (!rejectVisionUpdate) {
  // poseEstimator
  // .setVisionMeasurementStdDevs(VecBuilder.fill(STD_DEV_X_METERS,
  // STD_DEV_Y_METERS, STD_DEV_HEADING_RADS));
  // poseEstimator.addVisionMeasurement(visionPoseEstimate.pose,
  // visionPoseEstimate.timestampSeconds);
  // }
  // }

  public double getRotateAngleRadMT2() {
    Pose3d targetPoseRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(SHOOTER_LL_NAME); // pose of the target

    double targetX = targetPoseRobotSpace.getX(); // the forward offset between the center of the robot and target
    double targetY = targetPoseRobotSpace.getY(); // the sideways offset

    double targetOffsetRads = Math.atan2(targetY, targetX);

    return targetOffsetRads;
  }

  public double getDistanceToSpeakerMetersMT2() {
    Pose3d targetPoseRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(SHOOTER_LL_NAME);

    double x = targetPoseRobotSpace.getX();
    double y = targetPoseRobotSpace.getY();

    return Math.sqrt(x * x + y * y);
  }

  public double getOptimizedArmAngleRadsMT2() {
    return shooterMap.get(getDistanceToSpeakerMetersMT2());
  }
}