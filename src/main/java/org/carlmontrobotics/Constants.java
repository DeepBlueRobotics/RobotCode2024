// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.XboxController.Button;

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
	public static final class Drivetrain {
	}

	public static final class Arm {

		// Motor port
		public static final int ARM_MOTOR_PORT_MASTER = 13;
		public final static int ARM_MOTOR_PORT_FOLLOWER = 18;
		// Config for motors
		public static final boolean MOTOR_INVERTED_MASTER = false;
		public static final boolean MOTOR_INVERTED_FOLLOWER = true; //verifyed by design AND physical testing

		public static final double ROTATION_TO_RAD = 2 * Math.PI;
		public static final boolean ENCODER_INVERTED = true;

		public static final double ENCODER_OFFSET_RAD = 2.678 - 0.6095;

		// TODO: finish understand why this is broken public static final Measure<Angle>
		// INTAKE_ANGLE = Degrees.to(-1);

		// USE RADIANS FOR THE ARM
		public static final double INTAKE_ANGLE_RAD = Units.degreesToRadians(0);
		public static final double AMP_ANGLE_RAD = Units.degreesToRadians(105);
		public static final double SUBWOFFER_ANGLE_RAD = Units.degreesToRadians(24);
		public static final double SAFE_ZONE_ANGLE_RAD = Units.degreesToRadians(24);
		public static final double PODIUM_ANGLE_RAD = Units.degreesToRadians(24);
		public static final double CLIMBER_UP_ANGLE_RAD = Units.degreesToRadians(24);
		public static final double CLIMBER_DOWN_ANGLE_RAD = Units.degreesToRadians(24);

		// PID, Feedforward, Trapezoid
		public static final double kP = 50; //5.7938 / (2 * Math.PI);
		public static final double kI = 0;
		public static final double kD = 1.0761 / (2 * Math.PI);
		public static final double kS = 0.1498;
		public static final double kG = 0.3489;
		public static final double kV = 5.7539 / (2 * Math.PI);
		public static final double kA = 0.9569 / (2 * Math.PI);
		public static final double IZONE_RAD = 0;
		//fine for now, change it later before use - ("Incorect use of setIZone()" Issue #22)
		public static final double MAX_FF_VEL_RAD_P_S = 0.2; //rad/s
		public static final double MAX_FF_ACCEL_RAD_P_S =  53.728; // rad / s^2 ((.89*2)/(1.477/(61.875^2))/61.875)-20.84

		
		public static final double MIN_VOLTAGE = -12; //-((kS + kG + 1)/12);
		public static final double MAX_VOLTAGE = 12; //(kS + kG + 1)/12;

		// if needed
		public static final double COM_ARM_LENGTH_METERS = 0.381;
		public static final double ARM_MASS_KG = 9.59302503;

		public static TrapezoidProfile.Constraints TRAP_CONSTRAINTS;//initalized by arm constructor
		// other0;

		// public static final double MARGIN_OF_ERROR = Math.PI / 18;

		// Boundaries
		public static final double ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD = Math.pow(MAX_FF_VEL_RAD_P_S, 2) / MAX_FF_ACCEL_RAD_P_S;
		public static final double POS_TOLERANCE_RAD = Math.PI/512; // placeholder //Whether or not this is the actual account
															// idk TODO: test on actual encoder without a conversion
															// factor
		public static final double VEL_TOLERANCE_RAD_P_SEC = (POS_TOLERANCE_RAD/0.02); // 20ms per robot loop
		public static final double UPPER_ANGLE_LIMIT_RAD = 1.63;
		public static final double LOWER_ANGLE_LIMIT_RAD = -0.5;
		public static final double ARM_DISCONT_RAD = (LOWER_ANGLE_LIMIT_RAD + UPPER_ANGLE_LIMIT_RAD) / 2 - Math.PI;

		public static final double DISCONNECTED_ENCODER_TIMEOUT_SEC = 0.25;
		// Arm buttons

	}

	public static final class IntakeShooter {
		// in set() speed
		public static final double idleSpeed = 0;
		public static final double intakeSpeed = .7;
		public static final double outtakeSpeed = 1;

	}

	public static final class OI {
		public static final double JOY_THRESH = 0.01;

		public static final class Driver {
			public static final int port = 0;
		}

		public static final class Manipulator {
			public static final int port = 1;
			public static final int RAISE_TO_SPEAKER_POD_BUTTON = Button.kY.value;
			public static final int RAISE_TO_AMP_BUTTON = Button.kB.value;
			public static final int RAISE_TO_SPEAKER_SAFE_BUTTON = Button.kA.value;
			public static final int RAISE_TO_SPEAKER_NEXT_BUTTON = Button.kX.value;
			public static final int RAISE_TO_GROUND_BUTTON = Button.kStart.value;
			public static final int RAISE_TO_CLIMBER_BUTTON = Button.kLeftBumper.value;
			public static final int LOWER_TO_CLIMBER_BUTTON = Button.kRightBumper.value;
		}
	}
}
