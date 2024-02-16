package org.carlmontrobotics.subsystems;

import org.carlmontrobotics.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private final Drivetrain drivetrain;
  private final SwerveDrivePoseEstimator poseEstimator;
  private double tv, tx;

  //private double distOffset, horizOffset;
  //private double horizHeadingError, horizAdjust;

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
    //tracking();
    SmartDashboard.putNumber("angle offset", calcAngleOffset());
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
}