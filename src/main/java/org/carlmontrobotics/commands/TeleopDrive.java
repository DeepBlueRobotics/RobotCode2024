package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Drivetrainc.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.Robot;
import org.carlmontrobotics.subsystems.Drivetrain;
import static org.carlmontrobotics.RobotContainer.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopDrive extends Command {

  private static double robotPeriod = Robot.robot.getPeriod();
  private final Drivetrain drivetrain;
  private DoubleSupplier fwd;
  private DoubleSupplier str;
  private DoubleSupplier rcw;
  private BooleanSupplier slow;
  private double currentForwardVel = 0;
  private double currentStrafeVel = 0;
  private double prevTimestamp;
  private boolean babyMode;
  /**
   * Creates a new TeleopDrive.
   */
  public TeleopDrive(Drivetrain drivetrain, DoubleSupplier fwd, DoubleSupplier str, DoubleSupplier rcw,
      BooleanSupplier slow) {
    addRequirements(this.drivetrain = drivetrain);
    this.fwd = fwd;
    this.str = str;
    this.rcw = rcw;
    this.slow = slow;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartDashboard.putNumber("slow turn const", kSlowDriveRotation);
    // SmartDashboard.putNumber("slow speed const", kSlowDriveSpeed);
    // SmartDashboard.putNumber("normal turn const", kNormalDriveRotation);
    // SmartDashboard.putNumber("normal speed const", kNormalDriveSpeed);
    prevTimestamp = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = Timer.getFPGATimestamp();
    robotPeriod = currentTime - prevTimestamp;
    double[] speeds = getRequestedSpeeds();
    // SmartDashboard.putNumber("Elapsed time", currentTime - prevTimestamp);
    prevTimestamp = currentTime;
    babyMode = SmartDashboard.getBoolean("babymode", false);
    // kSlowDriveRotation = SmartDashboard.getNumber("slow turn const", kSlowDriveRotation);
    // kSlowDriveSpeed = SmartDashboard.getNumber("slow speed const", kSlowDriveSpeed);
    // kNormalDriveRotation = SmartDashboard.getNumber("normal turn const", kNormalDriveRotation);
    // kNormalDriveSpeed = SmartDashboard.getNumber("normal speed const", kNormalDriveSpeed);

    // SmartDashboard.putNumber("fwd", speeds[0]);
    // SmartDashboard.putNumber("strafe", speeds[1]);
    // SmartDashboard.putNumber("turn", speeds[2]);
    drivetrain.drive(speeds[0], speeds[1], speeds[2]);
  }

  public double[] getRequestedSpeeds() {
    // Sets all values less than or equal to a very small value (determined by the
    // idle joystick state) to zero.
    // Used to make sure that the robot does not try to change its angle unless it
    // is moving,
    double forward = fwd.getAsDouble();
    double strafe = str.getAsDouble();
    double rotateClockwise = rcw.getAsDouble();
    // SmartDashboard.putNumber("fwdIN", forward);
    // SmartDashboard.putNumber("strafeIN", strafe);
    // SmartDashboard.putNumber("turnIN", rotateClockwise);
    if (Math.abs(forward) <= Constants.OI.JOY_THRESH)
      forward = 0.0;
    else
      forward *= maxForward;
    if (Math.abs(strafe) <= Constants.OI.JOY_THRESH)
      strafe = 0.0;
    else
      strafe *= maxStrafe;
    if (Math.abs(rotateClockwise) <= Constants.OI.JOY_THRESH)
      rotateClockwise = 0.0;
    else
      rotateClockwise *= maxRCW;

    double driveMultiplier = slow.getAsBoolean() ? kSlowDriveSpeed : kNormalDriveSpeed;
    double rotationMultiplier = slow.getAsBoolean() ? kSlowDriveRotation : kNormalDriveRotation;
    if(babyMode == true){
      driveMultiplier = kBabyDriveSpeed;
      rotationMultiplier = kBabyDriveRotation;
    }
    // double driveMultiplier = kNormalDriveSpeed;
    // double rotationMultiplier = kNormalDriveRotation;

    forward *= driveMultiplier;
    strafe *= driveMultiplier;
    rotateClockwise *= rotationMultiplier;

    // Limit acceleration of the robot
    double accelerationX = (forward - currentForwardVel) / robotPeriod;
    double accelerationY = (strafe - currentStrafeVel) / robotPeriod;
    double translationalAcceleration = Math.hypot(accelerationX, accelerationY);
    // SmartDashboard.putNumber("Translational Acceleration", translationalAcceleration);
    if (translationalAcceleration > autoMaxAccelMps2 && false) {
      Translation2d limitedAccelerationVector = new Translation2d(autoMaxAccelMps2,
          Rotation2d.fromRadians(Math.atan2(accelerationY, accelerationX)));
      Translation2d limitedVelocityVector = limitedAccelerationVector.times(robotPeriod);
      currentForwardVel += limitedVelocityVector.getX();
      currentStrafeVel += limitedVelocityVector.getY();
    } else {
      currentForwardVel = forward;
      currentStrafeVel = strafe;
    }
    // SmartDashboard.putNumber("current velocity", Math.hypot(currentForwardVel, currentStrafeVel));

    // ATM, there is no rotational acceleration limit
    // currentForwardVel = forward;
    // currentStrafeVel = strafe;
    // If the above math works, no velocity should be greater than the max velocity,
    // so we don't need to limit it.

    return new double[] { currentForwardVel, currentStrafeVel, -rotateClockwise };
  }

  public boolean hasDriverInput() {
    return Math.abs(fwd.getAsDouble()) > Constants.OI.JOY_THRESH
        || Math.abs(str.getAsDouble()) > Constants.OI.JOY_THRESH
        || Math.abs(rcw.getAsDouble()) > Constants.OI.JOY_THRESH;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}