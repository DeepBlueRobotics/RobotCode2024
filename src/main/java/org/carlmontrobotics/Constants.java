
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

import org.carlmontrobotics.lib199.swerve.SwerveConfig;
import static org.carlmontrobotics.Config.CONFIG;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.util.Color;
import org.carlmontrobotics.subsystems.Led;

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

		public static final Color8Bit DEFAULT_COLOR_BLUE = new Color8Bit(0, 0, 200);
		public static final Color8Bit DETECT_NOTE_YELLOW = new Color8Bit(255, 255, 0);
		public static final Color8Bit HOLDING_GREEN = new Color8Bit(0, 250, 0);
		public static final Color8Bit RED_NEO_550_MOTOR = new Color8Bit(255,0,0);
		public static final int ledPort = 0;
		// TODO: figure out how to get port of LED, it could be 0 or
	}

	public static final class Effectorc {
		// PID values

		public static final int INTAKE = 0;
		public static final int OUTTAKE = 1;
		// 0.0001184
		public static final double[] kP = { 0, 0 /* 0.030717,0.0001 */ };
		public static final double[] kI = { /* /Intake/ */0, /* /Outake/ */0 };
		public static final double[] kD = { /* /Intake/ */0, /* /Outake/ */0 };
		public static final double[] kS = { /* /Intake/ */0.22, /* /Outake/ */0.29753 * 2 };
		public static final double[] kV = { 0.122, 0/* 0.065239, 0.077913 */ };
		public static final double[] kA = { 0, 0/* 0.0062809,0.05289 */ };
		public static final int INTAKE_PORT = 9; // port
		public static final int OUTAKE_PORT = 10; // port
		public static final int INTAKE_DISTANCE_SENSOR_PORT = 11; // port
		public static final int OUTAKE_DISTANCE_SENSOR_PORT = 10; // port
		public static final double DISTANCE_BETWEEN_SENSORS_INCHES = 8.189; // inches
		public static final double OFFSET_FROM_GROUND_INCHES = 21; // in
		public static final double DS_DEPTH_INCHES = 9.97; // Distance sensor Depth
		public static final double DETECT_DISTANCE_INCHES = 13;
		
		public static final double INTAKE_RPM = 6300;
		public static final double INTAKE_SLOWDOWN_RPM = 4500;
		public static final double MAX_SECONDS_OVERLOAD = 2.0;
		public static final double PASS_RPM = 2000;
		public static final double TEST_RPM = 3000;
		public static final double AMP_RPM = 1000;
		public static final double SUBWOOFER_RPM = 2100;
		public static final double PODIUM_RPM = 4000;
		public static final double SPEAKER_RPM = 2100;
		public static final double[] RPM_SELECTOR = {AMP_RPM, SUBWOOFER_RPM, PODIUM_RPM};
		// WTF FAB ISSUE
		//public static final double SAFE_RPM = 6000;// WTF FAB ISSUE

		public static final double EJECT_RPM_INTAKE = -2550;
		public static final double EJECT_RPM_OUTAKE = -2550;

		public static final double MANUAL_RPM_MAX = 9500;

		public static final double RPM_TOLERANCE = 200;
		public static final double SPEAKER_HEIGHT_INCHES = 83; // inches

		public static final boolean INTAKE_MOTOR_INVERSION = true;
		public static final boolean OUTAKE_MOTOR_INVERSION = false;

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
		public static final boolean MOTOR_INVERTED_FOLLOWER = true; // verifyed by design AND physical testing

		public static final double ROTATION_TO_RAD = 2 * Math.PI;
		public static final boolean ENCODER_INVERTED = true;

		// difference between CoG and arm is .328 rad
		public static final double ENCODER_OFFSET_RAD = -0.08 + .328; // - 0.6095;

		// TODO: finish understand why this is broken public static final Measure<Angle>
		// INTAKE_ANGLE = Degrees.to(-1);

		// USE RADIANS FOR THE ARM
		public static final double INTAKE_ANGLE_RAD = Units.degreesToRadians(0);
		public static final double HANG_ANGLE_RAD = Units.degreesToRadians(90);
		public static final double AMP_ANGLE_RAD = 1.28;
		public static final double AMP_ANGLE_RAD_NEW_MOTOR = 1.456;
		public static final double SPEAKER_ANGLE_RAD = -0.2;
		public static final double PODIUM_ANGLE_RAD = -0.2 + Units.degreesToRadians(7.5);
		// -0.427725
		public static final double GROUND_INTAKE_POS = -0.34537;
		public static final double HANG_ANGL_RAD = GROUND_INTAKE_POS + Units.degreesToRadians(30);

		public static final double SUBWOOFER_ANGLE_RAD = -0.22;// touching the base of the speaker
		public static final double SAFE_ZONE_ANGLE_RAD = Units.degreesToRadians(36);// touching the white line
		//public static final double PODIUM_ANGLE_RAD = Units.degreesToRadians(40);// touching the safe pad on the stage

		// 0.4 rad for shooting at podium

		// PID, Feedforward, Trapezoid
		public static final double kP = 0.45;// 5.7938; // (2 * Math.PI);
		public static final double kI = 0;
		public static final double kD = 0 * 1000;
		public static final double kS = 1.6 / 2; // 0.1498;
		public static final double kG = 0.8067; // 0.3489;
		public static final double kV = 5.1201 / (2 * Math.PI);
		public static final double kA = 0.43308 / (2 * Math.PI);
		public static final double IZONE_RAD = 0;
		// fine for now, change it later before use - ("Incorect use of setIZone()"
		// Issue #22)
		// public static final double MAX_FF_VEL_RAD_P_S = 0.2; //rad/s
		public static final double MAX_FF_VEL_RAD_P_S = Math.PI * .5;
		public static final double MAX_FF_ACCEL_RAD_P_S = 53.728 / 4; // rad / s^2
																		// ((.89*2)/(1.477/(61.875^2))/61.875)-20.84

		public static final double MAX_FF_VEL_RAD_P_S_BABY = 0;
		public static final double MAX_FF_ACCEL_RAD_P_S_BABY = 0;
		//TODO: determine these values^


		public static final double SOFT_LIMIT_LOCATION_IN_RADIANS = 0;
		public static final double CLIMB_POS = 1.701; //RADIANS
		public static final double MIN_VOLTAGE = -0.5; // -((kS + kG + 1)/12);
		public static final double MAX_VOLTAGE = 0.5; // (kS + kG + 1)/12;
		public static final double MIN_VOLTAGE_BABY = MIN_VOLTAGE/12 *0.7;
		public static final double MAX_VOLTAGE_BABY = MAX_VOLTAGE/12*0.7;
		public static final double CLIMB_FINISH_POS = -0.38;
		// if needed
		public static final double COM_ARM_LENGTH_METERS = 0.381;
		public static final double ARM_MASS_KG = 9.59302503;

		public static TrapezoidProfile.Constraints TRAP_CONSTRAINTS;// initalized by arm constructor
		// other0;

		// public static final double MARGIN_OF_ERROR = Math.PI / 18; v^2 /a

		// Boundaries
		public static final double ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD = 1.8345; // placeholder
		public static final double POS_TOLERANCE_RAD = Units.degreesToRadians(5); // placeholder //Whether or not this is the actual
																		// account
		// idk TODO: test on actual encoder without a conversion
		// factor
		public static final double VEL_TOLERANCE_RAD_P_SEC = (POS_TOLERANCE_RAD / 0.02); // 20ms per robot loop
		public static final double UPPER_ANGLE_LIMIT_RAD = 1.63;
		public static final double LOWER_ANGLE_LIMIT_RAD = -0.5;
		public static final double CLIMBER_UP_ANGLE_RAD = UPPER_ANGLE_LIMIT_RAD;
		public static final double CLIMBER_DOWN_ANGLE_RAD = LOWER_ANGLE_LIMIT_RAD;

		public static final double ARM_DISCONT_RAD = (LOWER_ANGLE_LIMIT_RAD + UPPER_ANGLE_LIMIT_RAD) / 2 - Math.PI;

		public static final double DISCONNECTED_ENCODER_TIMEOUT_SEC = 0.5;
		public static final double SMART_CURRENT_LIMIT_TIMEOUT = 0.8;
		// Arm buttons
	}

	public static final class Drivetrainc {

		// #region Subsystem Constants

		public static final double wheelBase = CONFIG.isSwimShady() ? Units.inchesToMeters(19.75)
				: Units.inchesToMeters(16.75);
		public static final double trackWidth = CONFIG.isSwimShady() ? Units.inchesToMeters(28.75)
				: Units.inchesToMeters(23.75);
		// "swerveRadius" is the distance from the center of the robot to one of the
		// modules
		public static final double swerveRadius = Math.sqrt(Math.pow(wheelBase / 2, 2) + Math.pow(trackWidth / 2, 2));
		// The gearing reduction from the drive motor controller to the wheels
		// Gearing for the Swerve Modules is 6.75 : 1
		public static final double driveGearing = 6.75;
		// Turn motor shaft to "module shaft"
		public static final double turnGearing = 150 / 7;

		public static final double driveModifier = 1;
		public static final double wheelDiameterMeters = Units.inchesToMeters(4.0) * 7.36 / 7.65 /*
																									 * empirical
																									 * correction
																									 */;
		public static final double mu = 1; /* 70/83.2; */

		public static final double NEOFreeSpeed = 5676 * (2 * Math.PI) / 60; // radians/s
		// Angular speed to translational speed --> v = omega * r / gearing
		public static final double maxSpeed = NEOFreeSpeed * (wheelDiameterMeters / 2.0) / driveGearing;
		public static final double maxForward = maxSpeed; // todo: use smart dashboard to figure this out
		public static final double maxStrafe = maxSpeed; // todo: use smart dashboard to figure this out
		// seconds it takes to go from 0 to 12 volts(aka MAX)
		public static final double secsPer12Volts = 0.1;

		// maxRCW is the angular velocity of the robot.
		// Calculated by looking at one of the motors and treating it as a point mass
		// moving around in a circle.
		// Tangential speed of this point mass is maxSpeed and the radius of the circle
		// is sqrt((wheelBase/2)^2 + (trackWidth/2)^2)
		// Angular velocity = Tangential speed / radius
		public static final double maxRCW = maxSpeed / swerveRadius;

		public static final boolean[] reversed = { false, false, false, false };
		// public static final boolean[] reversed = {true, true, true, true};
		// Determine correct turnZero constants (FL, FR, BL, BR)
		public static final double[] turnZeroDeg = RobotBase.isSimulation() ? new double[] {-90.0, -90.0, -90.0, -90.0 }
				: (CONFIG.isSwimShady() ? new double[] { 85.7812, 85.0782, -96.9433, -162.9492 }
						: new double[] { -48.6914, 63.3691, 94.1309, -6.7676 });/* real values here */

		// kP, kI, and kD constants for turn motor controllers in the order of
		// front-left, front-right, back-left, back-right.
		// Determine correct turn PID constants
		public static final double[] turnkP = { 51.078, 60.885, 60.946, 60.986 }; // {0.00374, 0.00374, 0.00374,
																					// 0.00374};
		public static final double[] turnkI = { 0, 0, 0, 0 };
		public static final double[] turnkD = { 0/* dont edit */, 0.5, 0.42, 1 }; // todo: use d
		// public static final double[] turnkS = {0.2, 0.2, 0.2, 0.2};
		public static final double[] turnkS = { 0.13027, 0.17026, 0.2, 0.23262 };

		// V = kS + kV * v + kA * a
		// 12 = 0.2 + 0.00463 * v
		// v = (12 - 0.2) / 0.00463 = 2548.596 degrees/s
		public static final double[] turnkV = { 2.6532, 2.7597, 2.7445, 2.7698 };
		public static final double[] turnkA = { 0.17924, 0.17924, 0.17924, 0.17924 };

		// kP is an average of the forward and backward kP values
		// Forward: 1.72, 1.71, 1.92, 1.94
		// Backward: 1.92, 1.92, 2.11, 1.89
		// Order of modules: (FL, FR, BL, BR)
		public static final double[] drivekP = CONFIG.isSwimShady() ? new double[] { 2.8, 2.8, 2.8, 2.8 }
				: new double[] { 1.75, 1.75, 1.75, .75 }; // {1.82/100, 1.815/100, 2.015/100,
																			// 1.915/100};
		public static final double[] drivekI = { 0, 0, 0, 0 };
		public static final double[] drivekD = { 0, 0, 0, 0 };
		public static final boolean[] driveInversion = (CONFIG.isSwimShady()
				? new boolean[] { false, false, false, false }
				: new boolean[] { true, false, true, false });
		public static final boolean[] turnInversion = { true, true, true, true };
		// kS
		public static final double[] kForwardVolts = { 0.26744, 0.31897, 0.27967, 0.2461 };
		public static final double[] kBackwardVolts = kForwardVolts;

		public static final double[] kForwardVels = { 2.81, 2.9098, 2.8378, 2.7391 };
		public static final double[] kBackwardVels = kForwardVels;
		public static final double[] kForwardAccels = { 1.1047 / 2, 0.79422 / 2, 0.77114 / 2, 1.1003 / 2 };
		public static final double[] kBackwardAccels = kForwardAccels;

		public static final double autoMaxSpeedMps = 0.35 * 4.4; // Meters / second
		public static final double autoMaxAccelMps2 = mu * g; // Meters / seconds^2
		public static final double autoMaxVolt = 10.0; // For Drivetrain voltage constraint in RobotPath.java
		// The maximum acceleration the robot can achieve is equal to the coefficient of
		// static friction times the gravitational acceleration
		// a = mu * 9.8 m/s^2
		public static final double autoCentripetalAccel = mu * g * 2;

		public static final boolean isGyroReversed = true;

		// PID values are listed in the order kP, kI, and kD
		public static final double[] xPIDController = CONFIG.isSwimShady() ? new double[] { 4, 0.0, 0.0 }
				: new double[] { 2, 0.0, 0.0 };
		public static final double[] yPIDController = xPIDController;
		public static final double[] thetaPIDController = CONFIG.isSwimShady() ? new double[] { 0.10, 0.0, 0.001 }
				: new double[] { 0.05, 0.0, 0.00 };

		public static final SwerveConfig swerveConfig = new SwerveConfig(wheelDiameterMeters, driveGearing, mu,
				autoCentripetalAccel, kForwardVolts, kForwardVels, kForwardAccels, kBackwardVolts, kBackwardVels,
				kBackwardAccels, drivekP, drivekI, drivekD, turnkP, turnkI, turnkD, turnkS, turnkV, turnkA, turnZeroDeg,
				driveInversion, reversed, driveModifier, turnInversion);

		// public static final Limelight.Transform limelightTransformForPoseEstimation =
		// Transform.BOTPOSE_WPIBLUE;

		// #endregion

		// #region Ports

		public static final int driveFrontLeftPort = CONFIG.isSwimShady() ? 8 : 11; //
		public static final int driveFrontRightPort = CONFIG.isSwimShady() ? 13 : 19; //
		public static final int driveBackLeftPort = CONFIG.isSwimShady() ? 5 : 14; //
		public static final int driveBackRightPort = CONFIG.isSwimShady() ? 11 : 17; // correct

		public static final int turnFrontLeftPort = CONFIG.isSwimShady() ? 7 : 12; //
		public static final int turnFrontRightPort = CONFIG.isSwimShady() ? 14 : 20; // 20
		public static final int turnBackLeftPort = CONFIG.isSwimShady() ? 6 : 15; //
		public static final int turnBackRightPort = CONFIG.isSwimShady() ? 12 : 16; // correct

		public static final int canCoderPortFL = CONFIG.isSwimShady() ? 4 : 0;
		public static final int canCoderPortFR = CONFIG.isSwimShady() ? 2 : 3;
		public static final int canCoderPortBL = CONFIG.isSwimShady() ? 3 : 2;
		public static final int canCoderPortBR = CONFIG.isSwimShady() ? 1 : 1;

		// #endregion

		// #region Command Constants

		public static double kNormalDriveSpeed = 1; // Percent Multiplier
		public static double kNormalDriveRotation = 0.5; // Percent Multiplier
		public static double kSlowDriveSpeed = 0.4; // Percent Multiplier
		public static double kSlowDriveRotation = 0.250; // Percent Multiplier

		//baby speed values, i just guessed the percent multiplier. TODO: find actual ones we wana use
		public static double kBabyDriveSpeed = 0.3;
		public static double kBabyDriveRotation = 0.2;
		public static double kAlignMultiplier = 1D / 3D;
		public static final double kAlignForward = 0.6;

		public static final double wheelTurnDriveSpeed = 0.0001; // Meters / Second ; A non-zero speed just used to
																	// orient the wheels to the correct angle. This
																	// should be very small to avoid actually moving the
																	// robot.

		public static final double[] positionTolerance = { Units.inchesToMeters(.5), Units.inchesToMeters(.5), 5 }; // Meters,
																													// Meters,
																													// Degrees
		public static final double[] velocityTolerance = { Units.inchesToMeters(1), Units.inchesToMeters(1), 5 }; // Meters,
																													// Meters,
																													// Degrees/Second

		// #endregion
		// #region Subsystem Constants

		// "swerveRadius" is the distance from the center of the robot to one of the
		// modules
		public static final double turnkP_avg = (turnkP[0] + turnkP[1] + turnkP[2] + turnkP[3]) / 4;
		public static final double turnIzone = .1;

		public static final double driveIzone = .1;

		public static final class Autoc {
			public static final ReplanningConfig replanningConfig = new ReplanningConfig( /*
																							 * put in
																							 * Constants.Drivetrain.Auto
																							 */
					false, // replan at start of path if robot not at start of path?
					false, // replan if total error surpasses total error/spike threshold?
					1.5, // total error threshold in meters that will cause the path to be replanned
					.8 // error spike threshold, in meters, that will cause the path to be replanned
			);
			public static final PathConstraints pathConstraints = new PathConstraints(1.54, 6.86, 2 * Math.PI,
					2 * Math.PI); // The constraints for this path. If using a differential drivetrain, the
									// angular constraints have no effect.
		}
		// #endregion
	}

	public static final class Limelightc {
		public static final String INTAKE_LL_NAME = "limelight-intake";
		public static final String SHOOTER_LL_NAME = "limelight-shooter";

		public static final double ERROR_TOLERANCE_RAD = 0.1;
		public static final double HORIZONTAL_FOV_DEG = 0;
		public static final double RESOLUTION_WIDTH_PIX = 640;
		public static final double MOUNT_ANGLE_DEG_SHOOTER = 42.5; 
		public static final double MOUNT_ANGLE_DEG_INTAKE = -22; // 23.228
		public static final double HEIGHT_FROM_GROUND_METERS_SHOOTER = Units.inchesToMeters(11.5); // 16.6
		public static final double HEIGHT_FROM_GROUND_METERS_INTAKE = Units.inchesToMeters(52); // 16.6
		public static final double ARM_TO_OUTTAKE_OFFSET_DEG = 115;
		public static final double NOTE_HEIGHT = Units.inchesToMeters(0);
		public static final double MIN_MOVEMENT_METERSPSEC = 0.5;
		public static final double MIN_MOVEMENT_RADSPSEC = 0.5;
		public static final double HEIGHT_FROM_RESTING_ARM_TO_SPEAKER_METERS = Units.inchesToMeters(65.5675);
		public static final double SIDEWAYS_OFFSET_TO_OUTTAKE_MOUTH = Units.inchesToMeters(19.5);
		public static final double END_EFFECTOR_BASE_ANGLE_RADS = Units.degreesToRadians(75);
		public static final double VERTICAL_OFFSET_FROM_ARM_PIVOT = Units.inchesToMeters(3.65);
		public static final class Apriltag {
			public static final int RED_SPEAKER_CENTER_TAG_ID = 4;
			public static final int BLUE_SPEAKER_CENTER_TAG_ID = 7;
			public static final double SPEAKER_CENTER_HEIGHT_METERS = Units.inchesToMeters(56.7); //88.125
			public static final double HEIGHT_FROM_BOTTOM_TO_SUBWOOFER = Units.inchesToMeters(26);
			public static final double HEIGHT_FROM_BOTTOM_TO_ARM_RESTING = Units.inchesToMeters(21.875);
		}
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
			// NEW BINDINGS(easier for manipulator)
			// Xbox left joy Y axis -> raw Intake control
			// Xbox right joy Y axis -> raw Outtake control
			// Xbox right trigger axis -> Intake pos + intake
			// Xbox left trigger axis -> amp pos , eject into amp
			// Xbox left bumper button -> CLOSE Speaker pos , Fire
			// Xbox right bumper button -> SAFE Speaker pos , Fire
			// Xbox X button -> goto Intake pos
			// Xbox Y button -> Eject rpm
			public static final Axis INTAKE_BUTTON = Axis.kLeftTrigger;
			public static final Axis SHOOTER_BUTTON = Axis.kRightTrigger;
			public static final int EJECT_BUTTON = Button.kLeftBumper.value;
			public static final int AMP_BUTTON = Button.kRightBumper.value;
			public static final Axis INTAKE_AX = Axis.kRightTrigger;
			public static final Axis AMP_AX = Axis.kLeftTrigger;
			public static final int SPEAKER_CLOSE = Button.kLeftBumper.value;
			public static final int SPEAKER_SAFE = Button.kRightBumper.value;
			public static final int SPEAKER_POS = Button.kX.value;
			// public static final int INTAKE_POS = Button.kX.value;
			public static final int EJECT_RPM = Button.kX.value;
			public static final int RAISE_CLIMBER = Button.kA.value;
			public static final int LOWER_CLIMBER = Button.kY.value;
			
		}

		public static final double JOY_THRESH = 0.01;
		public static final double MIN_AXIS_TRIGGER_VALUE = 0.2;// woah, this is high.
	}
}
