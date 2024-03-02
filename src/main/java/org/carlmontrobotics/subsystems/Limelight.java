package org.carlmontrobotics.subsystems;

import org.carlmontrobotics.Constants;

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
  //private final Drivetrain drivetrain;
  //private final SwerveDrivePoseEstimator poseEstimator;
  private double tv, tx;
  private double[] botPose = null;
  private double[] targetPose = null;
  private Pose3d botpose;

  private Arm arm;
  private IntakeShooter endEffector;

  //private double distOffset, horizOffset;
  //private double horizHeadingError, horizAdjust;

  public Limelight(Arm arm, IntakeShooter endEffector) {
    this.arm = arm;
    this.endEffector = endEffector;//needed for calculating rpm and firing angle

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

  // public Pose2d getCurrentPose(){
  //   System.out.println(poseEstimator.getEstimatedPosition());
  //   return poseEstimator.getEstimatedPosition();
  // }

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
      double distance = (Constants.Limelight.Apriltag.speakerCenterHeightMeters - Constants.Limelight.groundToCamYMeters) / Math.tan(angleToGoalRadians);
      SmartDashboard.putNumber("limelight distance", distance);
      return distance;
    }
    else{
      SmartDashboard.putNumber("limelight distance", -1);
      return -1;
    }
  }

  public double[] getFiringAngleRPM() {
      //consts
      double minRPM = 11000;//set to the max acheivable rpm of a free-load NEO550 Brushless
      double maxRPM = 11000;

      //not constant constants
      // double armAngle = arm.getArmPos();//flat to ground is zero
      double flatDist = distanceToTargetSpeaker() + camToArmJointXMeters;

      for (int i=0; i<maxRPM/10; i++){
        double rpm = 10*i;
        for (int i=MIN_ARM_ANGLE*5; i<MAX_ARM_ANGLE*5; i++){
          double armAngle = i/5


          /*
          PARAMETRIC:
          x,y of ring
          x = time*Fv*cos(Fa)
          y = speakerHeight = Fo_y + time*Fv*sin(Fa) - gravity*time^2   (gravity is 9.8m/s^2)
          */double fvConst = 2*Math.PI*Units.inchesToMeters(1)/60/*
          */double Fv = rpm * fvConst/*

          Fa (firing angle) = arm angle + shooter angle offset
          Fa = arm angle + 60 + 180 = ArmAngle + 240˚
                          ^ arm:intake angle is 120deg
          */double Fa = armAngle + 240/*

        Fo (firing offsetY) = (ArmJoint:limelight offsetY) + sin(armAngle)*armLength + sin(120˚)*EEffectorDepth/2
                                    ^ where arm starts        ^ where arm ends           ^ where shooter ends
        */double Fo_y = camToArmJointYMeters + Math.sin(armAngle)*ARM_LENGTH_METERS + Math.sin(Math.toRadians(120))/*

        PARAMETRIC:
        x,y of ring
        x = time*Fv*cos(Fa)
        y = speakerHeight = Fo_y + time*Fv*sin(Fa) - gravity*time^2   (gravity is 9.8m/s^2)

        or rather, time = flatDist / (Fv*cos(Fa))
        */double time = flatDist / (Fv*Math.cos(Fa))/*
        */double Fv = (speakerHeight - Fo_y + 9.8*time*time)/(time*Math.sin(Fa))/*

        Fv (firing velocity of surface of roller, per sec)
        Fv = circumfrence * rpm/60
        rpm = 60* Fv/circumfrence
        */double rpm = 60 * Fv/ROLLER_CIRCUMFRENCE;/*


        OUTPUTS
        kD is kDrag
        rpm = kD*t^2 + rpm
        */
      }

      for(int i = 0; i<= 360; i++) {
          double t = Math.sqrt((OFFSETFROMGROUND-SpeakerHeight+distance*Math.tan(i)));
          double rpm = distance/Math.cos(i)*t;
          if(rpm<minRPM) {
              minRPM = rpm;
          }
      }
      if(minRPM == Integer.MAX_VALUE) {
          System.err.println("FAILURE");
      }
      return minRPM;
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


  functions:
  getBotpose3d vs getCurrentPose

  */
}
