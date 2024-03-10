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
	public static final double g = 9.81; //meters per second squared

	public static final class Led {
		public static final int ledLength = 5;

		public static final Color8Bit defaultColor = new Color8Bit(0, 0, 200);
		public static final Color8Bit detectNote = new Color8Bit(250,140,3);
		public static final Color8Bit holding = new Color8Bit(0,250,0);
		public static final Color8Bit ejectColor = new Color8Bit(250,250,0);
		public static final int ledPort = 0;
		public static final Color8Bit intakeColor = new Color8Bit(154,0,255); //intake detection = purple
		public static final Color8Bit outtakeColor = new Color8Bit(0,9,255); //outtake detection = blue
		public static final Color8Bit intakeouttakeColor = new Color8Bit(0,255,9); //intake and outtake (both) = green
		//Red when nothing, purple/blue when intake/outtake detect only, green when both
	}

	public static final class Effectorc {
		// PID values
		public static final int INTAKE = 0;
		public static final int OUTTAKE = 1;

		public static final double[] kP = {/*/Intake/*/ 0.0001, /*/Outake/*/0};
		public static final double[] kI = {/*/Intake/*/0.00001, /*/Outake/*/0};
		public static final double[] kD = {/*/Intake/*/0, /*/Outake/*/0 };
		public static final double[] kS = {/*/Intake/*/0.29753, /*/Outake/*/0.29753};
		public static final double[] kV = {/*/Intake/*/0.077913, /*/Outake/*/0.077913};
		public static final double[] kA = {/*/Intake/*/0.05289, /*/Outake/*/0.05289};
		public static final int INTAKE_PORT = 0; //port
		public static final int OUTAKE_PORT = 1; //port
		public static final int INTAKE_DISTANCE_SENSOR_PORT = 10; //port
		public static final int OUTAKE_DISTANCE_SENSOR_PORT = 11; //port
		public static final double DISTANCE_BETWEEN_SENSORS_INCHES = 6.5; // inches
		public static final double OFFSET_FROM_GROUND_INCHES = 21; // in
		public static final double DS_DEPTH_INCHES = 9.97; // Distance sensor Depth
		public static final double DETECT_DISTANCE_INCHES = 13;

		public static final double OUTAKE_RPM_CLOSE = 4000;
		public static final double OUTAKE_RPM_SAFE = 7000;

		public static final double INTAKE_RPM = 6000;
		public static final double INTAKE_SLOWDOWN_RPM = 1500;

		public static final double PASS_RPM = 750;

		public static final double AMP_RPM = 1500;
		public static final double SPEAKER_RPM = 6000;

		public static final double EJECT_RPM_INTAKE = 750;
		public static final double EJECT_RPM_OUTAKE = 750;

		public static final double MANUAL_RPM_MAX = 4000;


		public static final double RPM_TOLERANCE = 20;
		public static final double SPEAKER_HEIGHT_INCHES = 83; // inches

		public static final boolean INTAKE_MOTOR_INVERSION = false;
		public static final boolean OUTAKE_MOTOR_INVERSION = true;

		public static final int EJECT_TIME_SECS = 5;
		public static final int INTAKE_TIME_SECS = 4;

		public static final int ledLength = 85;
		public static final int ledPort = 85;
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
		public static final boolean ENCODER_INVERTED = false;

		public static final int MAX_VOLTAGE = 12;
		public static final double ENCODER_OFFSET_RAD = 0;

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
		public static final double kP = 0.1;
		public static final double kI = 0.1;
		public static final double kD = 0.1;
		public static final double kS = 0.1;
		public static final double kG = 0.1;
		public static final double kV = 0.1;
		public static final double kA = 0.1;
		public static final double IZONE_RAD = .09;
		//fine for now, change it later before use - ("Incorect use of setIZone()" Issue #22)
		public static final double MAX_FF_VEL_RAD_P_S = 9.92; //rad/s Aarav did the work
		public static final double MAX_FF_ACCEL_RAD_P_S =  .0183; // rad / s^2 Aarav did the math

		// if needed
		public static final double COM_ARM_LENGTH_METERS = 0.381;
		public static final double ARM_MASS_KG = 9.59302503;

		public static TrapezoidProfile.Constraints TRAP_CONSTRAINTS;//initalized by arm constructor
		// other0;

		// public static final double MARGIN_OF_ERROR = Math.PI / 18;

		// Boundaries
		public static final double ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD = 1.8345; // placeholder
		public static final double POS_TOLERANCE_RAD = Math.PI/512; // placeholder //Whether or not this is the actual account
															// idk TODO: test on actual encoder without a conversion
															// factor
		public static final double VEL_TOLERANCE_RAD_P_SEC = (POS_TOLERANCE_RAD/0.02); // 20ms per robot loop
		public static final double UPPER_ANGLE_LIMIT_RAD = Units.degreesToRadians(70);
		public static final double LOWER_ANGLE_LIMIT_RAD = Units.degreesToRadians(0);
		public static final double ARM_DISCONT_RAD = (LOWER_ANGLE_LIMIT_RAD + UPPER_ANGLE_LIMIT_RAD) / 2 - Math.PI;

	}

	public static final class Limelight {
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
			public static final int INTAKE = Axis.kRightTrigger.value;
			public static final int AMP = Axis.kLeftTrigger.value;
			public static final int SPEAKER_CLOSE = Button.kLeftBumper.value;
			public static final int SPEAKER_SAFE = Button.kRightBumper.value;
			public static final int SPEAKER_POS = Button.kA.value;
			public static final int INTAKE_POS = Button.kX.value;
			public static final int EJECT_RPM = Button.kY.value;
		}
		public static final double JOY_THRESH = 0.01;
        public static final double MIN_AXIS_TRIGGER_VALUE = 0.2;//woah, this is high.
	}
}
