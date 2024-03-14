
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units.*;
import org.carlmontrobotics.lib199.Limelight;
import org.carlmontrobotics.lib199.Limelight.Transform;
import org.carlmontrobotics.lib199.swerve.SwerveConfig;

import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final double g = 9.81; // meters per second squared

	public static final class Led {
		public static final int ledLength = 1000;//lower doesn't work but higher does
		public static final int ledPort = 5;

		public static final Color8Bit defaultColor = new Color8Bit(0, 0, 200);
		public static final Color8Bit detectNote = new Color8Bit(250,140,3);
		public static final Color8Bit holding = new Color8Bit(0,250,0);
		public static final Color8Bit ejectColor = new Color8Bit(250,250,0);
		
		public static final Color8Bit intakeColor = new Color8Bit(154,0,255); //intake detection = purple
		public static final Color8Bit outtakeColor = new Color8Bit(0,9,255); //outtake detection = blue
		public static final Color8Bit intakeouttakeColor = new Color8Bit(0,255,9); //intake and outtake (both) = green
		//Red when nothing, purple/blue when intake/outtake detect only, green when both
			
	}

	public static final class Effectorc {
		// PID values

		public static final int INTAKE = 0;
		public static final int OUTTAKE = 1;

		public static final double[] kP = { /* /Intake/ */ 0.0001184, /* /Outake/ */0.0001 };
		public static final double[] kI = { /* /Intake/ */0, /* /Outake/ */0 };
		public static final double[] kD = { /* /Intake/ */0, /* /Outake/ */0 };
		public static final double[] kS = { /* /Intake/ */0.52174, /* /Outake/ */0.29753 * 2 };
		public static final double[] kV = { /* /Intake/ */0.064142, /* /Outake/ */0.077913 };
		public static final double[] kA = { /* /Intake/ */0.0074649, /* /Outake/ */0.05289 };
		public static final int INTAKE_PORT = 9; // port
		public static final int OUTAKE_PORT = 10; // port
		public static final int INTAKE_DISTANCE_SENSOR_PORT = 11; // port
		public static final int OUTAKE_DISTANCE_SENSOR_PORT = 10; // port
		public static final double DISTANCE_BETWEEN_SENSORS_INCHES = 8.189; // inches
		public static final double OFFSET_FROM_GROUND_INCHES = 21; // in
		public static final double DS_DEPTH_INCHES = 9.97; // Distance sensor Depth
		public static final double DETECT_DISTANCE_INCHES = 13;
		
		public static final double INTAKE_RPM = 7500;
		public static final double INTAKE_SLOWDOWN_RPM = 1000;

		public static final double PASS_RPM = 5800;

		public static final double AMP_RPM = 1500;
		public static final double SPEAKER_RPM = 4500;
		public static final double SUBWOOFER_RPM = 6000;//WTF FAB ISSUE
		public static final double SAFE_RPM = 6000;//WTF FAB ISSUE

		public static final double EJECT_RPM_INTAKE = -2550;
		public static final double EJECT_RPM_OUTAKE = -2550;

		public static final double MANUAL_RPM_MAX = 9500;


		public static final double RPM_TOLERANCE = 200;
		public static final double SPEAKER_HEIGHT_INCHES = 83; // inches

		public static final boolean INTAKE_MOTOR_INVERSION = true;
		public static final boolean OUTAKE_MOTOR_INVERSION = true;

		public static final double EJECT_TIME_SECS = 5.;
		public static final double EJECT_MIN_SECS = 1.25;
		public static final double INTAKE_TIME_SECS = 4.;
        public static final double ledDefaultColorRestoreTime = 3;
	    public static final Color defaultColor = new Color(0, 0, 200);
        public static final Color pickupSuccessColor = new Color(0, 200, 0);

	}

	public static final class Armc {

		// Motor port
		public static final int ARM_MOTOR_PORT_MASTER = 13;
		public final static int ARM_MOTOR_PORT_FOLLOWER = 18;
		// Config for motors
		public static final boolean MOTOR_INVERTED_MASTER = false;
		public static final boolean MOTOR_INVERTED_FOLLOWER = true; //verifyed by design AND physical testing

		public static final double ROTATION_TO_RAD = 2 * Math.PI;
		public static final boolean ENCODER_INVERTED = true;

		public static final double ENCODER_OFFSET_RAD = 2.678; //- 0.6095;

		// TODO: finish understand why this is broken public static final Measure<Angle>
		// INTAKE_ANGLE = Degrees.to(-1);

		// USE RADIANS FOR THE ARM
		public static final double INTAKE_ANGLE_RAD = Units.degreesToRadians(0);
		public static final double AMP_ANGLE_RAD = Units.degreesToRadians(90);

		public static final double SUBWOOFER_ANGLE_RAD = Units.degreesToRadians(0);//touching the base of the speaker
		public static final double SAFE_ZONE_ANGLE_RAD = Units.degreesToRadians(36);//touching the white line
		public static final double PODIUM_ANGLE_RAD = Units.degreesToRadians(40);//touching the safe pad on the stage

		// PID, Feedforward, Trapezoid
		public static final double kP = 5.7938;
		public static final double kI = 0;
		public static final double kD = 1.0761 * 1000;
		public static final double kS = 0.1498;
		public static final double kG = 0.3489;
		public static final double kV = 5.7539 / (2 * Math.PI);
		public static final double kA = 0.9569 / (2 * Math.PI);
		public static final double IZONE_RAD = 0;
		//fine for now, change it later before use - ("Incorect use of setIZone()" Issue #22)
		//public static final double MAX_FF_VEL_RAD_P_S = 0.2; //rad/s
		public static final double MAX_FF_ACCEL_RAD_P_S =  53.728; // rad / s^2 ((.89*2)/(1.477/(61.875^2))/61.875)-20.84


		public static final double MIN_VOLTAGE = -3; //-((kS + kG + 1)/12) +2 becaus why not;
		public static final double MAX_VOLTAGE = 3; //(kS + kG + 1)/12 +2;

		// if needed
		public static final double COM_ARM_LENGTH_METERS = 0.381;
		public static final double ARM_MASS_KG = 9.59302503;

		public static TrapezoidProfile.Constraints TRAP_CONSTRAINTS;//initalized by arm constructor
		// other0;

		// public static final double MARGIN_OF_ERROR = Math.PI / 18; v^2 /a

		// Boundaries
		public static final double ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD = 1.8345; // placeholder
		public static final double POS_TOLERANCE_RAD = Math.PI/512; // placeholder //Whether or not this is the actual account
															// idk TODO: test on actual encoder without a conversion
															// factor
		public static final double VEL_TOLERANCE_RAD_P_SEC = (POS_TOLERANCE_RAD/0.02); // 20ms per robot loop
		public static final double UPPER_ANGLE_LIMIT_RAD = 1.63;
		public static final double LOWER_ANGLE_LIMIT_RAD = -0.5;
		public static final double CLIMBER_UP_ANGLE_RAD = UPPER_ANGLE_LIMIT_RAD;
		public static final double CLIMBER_DOWN_ANGLE_RAD = LOWER_ANGLE_LIMIT_RAD;

		public static final double ARM_DISCONT_RAD = (LOWER_ANGLE_LIMIT_RAD + UPPER_ANGLE_LIMIT_RAD) / 2 - Math.PI;

		public static final double DISCONNECTED_ENCODER_TIMEOUT_SEC = 0.25;
		// Arm buttons
		}
		public static final class Drivetrainc {

				
				//#region Subsystem Constants

			public static final double wheelBase = Units.inchesToMeters(16.75);
			public static final double trackWidth = Units.inchesToMeters(23.75);
			// "swerveRadius" is the distance from the center of the robot to one of the modules
			public static final double swerveRadius = Math.sqrt(Math.pow(wheelBase / 2, 2) + Math.pow(trackWidth / 2, 2));
			// The gearing reduction from the drive motor controller to the wheels
			// Gearing for the Swerve Modules is 6.75 : 1
			public static final double driveGearing = 6.75;
			// Turn motor shaft to "module shaft"
			public static final double turnGearing = 150 / 7;

			public static final double driveModifier = 1;
			public static final double wheelDiameterMeters = Units.inchesToMeters(4.0) * 7.36/7.65 /* empirical correction */;
			public static final double mu = 1; /* 70/83.2;  */

			public static final double NEOFreeSpeed = 5676 * (2 * Math.PI) / 60;    // radians/s
			// Angular speed to translational speed --> v = omega * r / gearing
			public static final double maxSpeed = NEOFreeSpeed * (wheelDiameterMeters / 2.0) / driveGearing;
			public static final double maxForward = maxSpeed; //todo: use smart dashboard to figure this out
			public static final double maxStrafe = maxSpeed; // todo: use smart dashboard to figure this out
		 	// seconds it takes to go from 0 to 12 volts(aka MAX)
			public static final double secsPer12Volts = 0.1;

			// maxRCW is the angular velocity of the robot.
			// Calculated by looking at one of the motors and treating it as a point mass moving around in a circle.
			// Tangential speed of this point mass is maxSpeed and the radius of the circle is sqrt((wheelBase/2)^2 + (trackWidth/2)^2)
			// Angular velocity = Tangential speed / radius
			public static final double maxRCW = maxSpeed / swerveRadius;

			public static final boolean[] reversed = {false, false, false, false};
			// public static final boolean[] reversed = {true, true, true, true};
			// Determine correct turnZero constants (FL, FR, BL, BR)
			public static final double[] turnZeroDeg = RobotBase.isSimulation() ?
					new double[] {0, 0, 0, 0} :
					new double[] {-48.6914, 63.3691, 94.1309, -6.7676};/*real values here*/

			// kP, kI, and kD constants for turn motor controllers in the order of front-left, front-right, back-left, back-right.
			// Determine correct turn PID constants
			public static final double[] turnkP = {51.078,60.885,60.946,60.986}; //{0.00374, 0.00374, 0.00374, 0.00374};
			public static final double[] turnkI = {0, 0, 0, 0};
			public static final double[] turnkD = {0/*dont edit */, 0.5, 0.42, 1}; // todo: use d
			//public static final double[] turnkS = {0.2, 0.2, 0.2, 0.2};
			public static final double[] turnkS = {0.13027, 0.17026, 0.2, 0.23262};

			// V = kS + kV * v + kA * a
			// 12 = 0.2 + 0.00463 * v
			// v = (12 - 0.2) / 0.00463 = 2548.596 degrees/s	
			public static final double[] turnkV = {2.6532, 2.7597, 2.7445, 2.7698};
			public static final double[] turnkA = {0.17924, 0.17924, 0.17924, 0.17924};

			// kP is an average of the forward and backward kP values
			// Forward: 1.72, 1.71, 1.92, 1.94
			// Backward: 1.92, 1.92, 2.11, 1.89
			// Order of modules: (FL, FR, BL, BR)
			public static final double[] drivekP = {2.8, 2.8, 2.8, 2.8}; //{1.82/100, 1.815/100, 2.015/100, 1.915/100};
			public static final double[] drivekI = {0, 0, 0, 0};
			public static final double[] drivekD = {0, 0, 0, 0};
			public static final boolean[] driveInversion = {true, false, true, false};
			public static final boolean[] turnInversion = {true, true, true, true};
			// kS
			public static final double[] kForwardVolts = {0.26744, 0.31897, 0.27967, 0.2461};
			public static final double[] kBackwardVolts = kForwardVolts;

			public static final double[] kForwardVels = {2.81, 2.9098, 2.8378, 2.7391};
			public static final double[] kBackwardVels = kForwardVels;
			public static final double[] kForwardAccels = {1.1047/2, 0.79422/2, 0.77114/2, 1.1003/2};
			public static final double[] kBackwardAccels = kForwardAccels;

			public static final double autoMaxSpeedMps = 0.35 * 4.4;  // Meters / second
			public static final double autoMaxAccelMps2 = mu * g;  // Meters / seconds^2
			public static final double autoMaxVolt = 10.0;   // For Drivetrain voltage constraint in RobotPath.java
			// The maximum acceleration the robot can achieve is equal to the coefficient of static friction times the gravitational acceleration
			// a = mu * 9.8 m/s^2
			public static final double autoCentripetalAccel = mu * g * 2;

			public static final boolean isGyroReversed = true;

			// PID values are listed in the order kP, kI, and kD
			public static final double[] xPIDController = {4, 0.0, 0.0};
			public static final double[] yPIDController = {4, 0.0, 0.0};
			public static final double[] thetaPIDController = {0.10, 0.0, 0.001};

			public static final SwerveConfig swerveConfig = new SwerveConfig(wheelDiameterMeters, driveGearing, mu, autoCentripetalAccel, kForwardVolts, kForwardVels, kForwardAccels, kBackwardVolts, kBackwardVels, kBackwardAccels, drivekP, drivekI, drivekD, turnkP, turnkI, turnkD, turnkS, turnkV, turnkA, turnZeroDeg, driveInversion, reversed, driveModifier, turnInversion);

			public static final Limelight.Transform limelightTransformForPoseEstimation = Transform.BOTPOSE_WPIBLUE;

			//#endregion

			//#region Ports

			public static final int driveFrontLeftPort = 11; //
			public static final int driveFrontRightPort = 19; // 	
			public static final int driveBackLeftPort = 14; //
			public static final int driveBackRightPort = 17; //correct

			public static final int turnFrontLeftPort = 12; //
			public static final int turnFrontRightPort = 20; // 20
			public static final int turnBackLeftPort = 15; // 
			public static final int turnBackRightPort = 16; //correct

			public static final int canCoderPortFL = 0;
			public static final int canCoderPortFR = 3;
			public static final int canCoderPortBL = 2;
			public static final int canCoderPortBR = 1;

			//#endregion

			//#region Command Constants

			public static  double kNormalDriveSpeed = 1; // Percent Multiplier
			public static  double kNormalDriveRotation = 0.5; // Percent Multiplier
			public static  double kSlowDriveSpeed = 0.4; // Percent Multiplier
			public static  double kSlowDriveRotation = 0.250; // Percent Multiplier
			public static  double kAlignMultiplier = 1D/3D;
			public static final double kAlignForward = 0.6;

			public static final double wheelTurnDriveSpeed = 0.0001; // Meters / Second ; A non-zero speed just used to orient the wheels to the correct angle. This should be very small to avoid actually moving the robot.

			public static final double[] positionTolerance = {Units.inchesToMeters(.5), Units.inchesToMeters(.5), 5}; // Meters, Meters, Degrees
			public static final double[] velocityTolerance = {Units.inchesToMeters(1), Units.inchesToMeters(1), 5}; // Meters, Meters, Degrees/Second

			//#endregion
			//#region Subsystem Constants

				// "swerveRadius" is the distance from the center of the robot to one of the modules
				public static final double turnkP_avg = (turnkP[0]+turnkP[1]+turnkP[2]+turnkP[3])/4;
				public static final double turnIzone = .01;

				public static final double driveIzone = .01;

				public static final class Auto {
					public static final ReplanningConfig replanningConfig = new ReplanningConfig( /*put in Constants.Drivetrain.Auto*/
						false, //replan at start of path if robot not at start of path?
						false, //replan if total error surpasses total error/spike threshold?
						1.5, //total error threshold in meters that will cause the path to be replanned
						.8 //error spike threshold, in meters, that will cause the path to be replanned
					);
				}
				//#endregion
		}

		public static final class OI {
			public static final class Driver {
				public static final int port = 0;

				public static final int slowDriveButton = Button.kLeftBumper.value;
				public static final int resetFieldOrientationButton = Button.kRightBumper.value;
				public static final int toggleFieldOrientedButton = Button.kStart.value;

				public static final int rotateFieldRelative0Deg = Button.kY.value;
				public static final int rotateFieldRelative90Deg = Button.kB.value;
				public static final int rotateFieldRelative180Deg = Button.kA.value;
				public static final int rotateFieldRelative270Deg = Button.kX.value;
			}

		public static final class Manipulator {
			public static final int port = 1;
			//NEW BINDINGS(easier for manipulator)
			//Xbox left joy Y axis -> raw Intake control
			//Xbox right joy Y axis -> raw Outtake control
			//Xbox right trigger axis -> Intake pos + intake
			//Xbox left trigger axis -> amp pos , eject into amp
			//Xbox left bumper button -> CLOSE Speaker pos , Fire
			//Xbox right bumper button -> SAFE  Speaker pos , Fire
			//Xbox X button -> goto Intake pos
			//Xbox Y button -> Eject rpm
			public static final Axis INTAKE_BUTTON = Axis.kLeftTrigger;
			public static final Axis SHOOTER_BUTTON = Axis.kRightTrigger;
			public static final int EJECT_BUTTON = Button.kLeftBumper.value;
			public static final int AMP_BUTTON = Button.kRightBumper.value;
			public static final Axis INTAKE_AX = Axis.kRightTrigger;
			public static final Axis AMP_AX = Axis.kLeftTrigger;
			public static final int SPEAKER_CLOSE = Button.kLeftBumper.value;
			public static final int SPEAKER_SAFE = Button.kRightBumper.value;
			public static final int SPEAKER_POS = Button.kA.value;
			public static final int INTAKE_POS = Button.kX.value;
			public static final int EJECT_RPM = Button.kY.value;
			public static final int RAISE_CLIMBER = Button.kLeftStick.value;
			public static final int LOWER_CLIMBER = Button.kRightStick.value;
		}
		public static final double JOY_THRESH = 0.01;
    public static final double MIN_AXIS_TRIGGER_VALUE = 0.2;//woah, this is high.
	}
}
