package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.Drivetrainc.*;
import static org.carlmontrobotics.Constants.Limelightc.*;
import static org.carlmontrobotics.Constants.Limelightc.Apriltag.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private final Drivetrain drivetrain;

  private final InterpolatingDoubleTreeMap shooterMap;

  public Limelight(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

    LimelightHelpers.SetFiducialIDFiltersOverride(SHOOTER_LL_NAME, VALID_IDS);

    shooterMap = new InterpolatingDoubleTreeMap(); // add values after testing
    // key is distance (meters), value is angle (rads)
    shooterMap.put(2.06, .01);
    shooterMap.put(3.05, 0.26);
    shooterMap.put(2.5, 0.23);
    shooterMap.put(1.39, -0.18);
    shooterMap.put(3.45, 0.3);
    shooterMap.put(3.1, 0.3);

    SmartDashboard.putBoolean("shooter sees note", false);

    // ASSUMING SHOOTING AT 4000 RPM
    // changing speed multipliers for auto intaking note
    // SmartDashboard.putNumber("forward speed multiplier", 1.5);
    // SmartDashboard.putNumber("strafe speed multiplier", 1.5);
    // SmartDashboard.putNumber("rotational speed multiplier", 2);

    // tuning apriltag alignment pid and tolerances
    // SmartDashboard.putNumber("rotation to align", getRotateAngleRadMT2());

    // SmartDashboard.putNumber("apriltag align kp", thetaPIDController[0]);
    // SmartDashboard.putNumber("apriltag align ki", thetaPIDController[1]);
    // SmartDashboard.putNumber("apriltag align kd", thetaPIDController[2]);

    // SmartDashboard.putNumber("apriltag align pos tolerance",
    // positionTolerance[2]);
    // SmartDashboard.putNumber("apriltag align vel tolerance",
    // velocityTolerance[2]);

  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("shooter sees note", LimelightHelpers.getTV(SHOOTER_LL_NAME));

    // intake limelight testing
    // SmartDashboard.putBoolean("see note",
    // LimelightHelpers.getTV(INTAKE_LL_NAME));
    // SmartDashboard.putNumber("distance to note", getDistanceToNoteMeters());
    // SmartDashboard.putNumber("intake tx",
    // LimelightHelpers.getTX(INTAKE_LL_NAME));

    // shooter limelight testing
    // SmartDashboard.putNumber("distance to speaker (meters)",
    // getDistanceToSpeakerMetersMT2());
    // SmartDashboard.putNumber("optimized arm angle",
    // getOptimizedArmAngleRadsMT2());
  }

  public double getTXDeg(String limelightName) {
    return (limelightName == INTAKE_LL_NAME)
        ? LimelightHelpers.getTX(INTAKE_LL_NAME)
        : -LimelightHelpers.getTY(SHOOTER_LL_NAME);
  }

  public double getTYDeg(String limelightName) {
    return (limelightName == INTAKE_LL_NAME)
        ? LimelightHelpers.getTY(INTAKE_LL_NAME)
        : LimelightHelpers.getTX(SHOOTER_LL_NAME);
  }

  public double getDistanceToSpeakerMeters() {
    if (LimelightHelpers
        .getFiducialID(SHOOTER_LL_NAME) == RED_SPEAKER_CENTER_TAG_ID
        || LimelightHelpers
            .getFiducialID(SHOOTER_LL_NAME) == BLUE_SPEAKER_CENTER_TAG_ID) {
      Rotation2d angleToGoal = Rotation2d.fromDegrees(MOUNT_ANGLE_DEG_SHOOTER)
          .plus(Rotation2d.fromDegrees(getTYDeg(SHOOTER_LL_NAME))); // because limelight is mounted
                                                                    // horizontally
      double distance = (SPEAKER_CENTER_HEIGHT_METERS - HEIGHT_FROM_GROUND_METERS_SHOOTER)
          / angleToGoal.getTan();
      // SmartDashboard.putNumber("limelight distance", distance);
      return distance;
    } else {
      // SmartDashboard.putNumber("limelight distance", -1);
      return -1;
    }
  }

  public double getDistanceToNoteMeters() {
    Rotation2d angleToGoal = Rotation2d.fromDegrees(MOUNT_ANGLE_DEG_INTAKE)
        .plus(Rotation2d.fromDegrees(getTYDeg(INTAKE_LL_NAME)));
    if (angleToGoal.getDegrees() <= 0) {
      double distance = (HEIGHT_FROM_GROUND_METERS_INTAKE - NOTE_HEIGHT)
          / Math.tan(Math.abs(angleToGoal.getRadians()));
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
    return END_EFFECTOR_BASE_ANGLE_RADS - Math
        .atan(armRestingHeightToSubwooferMeters / horizontalDistanceMeters);
  }

  public double getRotateAngleRadMT2() {
    Pose3d targetPoseRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(SHOOTER_LL_NAME); // pose of the target

    double targetX = targetPoseRobotSpace.getX(); // the forward offset between the center of the
                                                  // robot and target
    double targetZ = -targetPoseRobotSpace.getZ(); // the sideways offset

    double targetOffsetRads = MathUtil.inputModulus(Math.atan2(targetX, targetZ), -Math.PI, Math.PI);

    return targetOffsetRads;
  }

  public double getDistanceToSpeakerMetersMT2() {
    Pose3d targetPoseRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(SHOOTER_LL_NAME);

    double x = targetPoseRobotSpace.getX();
    double z = targetPoseRobotSpace.getZ();

    return Math.hypot(x, z);
  }

  public double getOptimizedArmAngleRadsMT2() {
    return shooterMap.get(getDistanceToSpeakerMetersMT2());
  }

  public boolean seesTag() {
    return LimelightHelpers.getTV(SHOOTER_LL_NAME);
  }
}
