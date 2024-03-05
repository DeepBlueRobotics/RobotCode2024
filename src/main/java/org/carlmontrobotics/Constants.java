// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

import edu.wpi.first.wpilibj.XboxController.Axis;
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

	public static final class Drivetrain {
	}

	public static final class Arm{}

	public static final class IntakeShoot {
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
		public static final double DISTANCE_BETWEEN_SENSORS_INCHES = 8.189; // inches
		public static final double OFFSET_FROM_GROUND_INCHES = 21; // in
		public static final double DS_DEPTH_INCHES = 9.97; // Distance sensor Depth
		public static final double DETECT_DISTANCE_INCHES = 13;

		public static final double OUTAKE_RPM = 6000;
		public static final double INTAKE_RPM = -6000;
		public static final double INTAKE_SLOWDOWN_RPM = -1.0;

		public static final double PASS_RPM = 3000;

		public static final double AMP_RPM = 1500;
		public static final double SPEAKER_RPM = 6000;

		public static final double EJECT_RPM_INTAKE = 3000;
		public static final double EJECT_RPM_OUTAKE = 3000;


		public static final double RPM_TOLERANCE = 10;
		public static final double SPEAKER_HEIGHT_INCHES = 83; // inches 

		public static final boolean INTAKE_MOTOR_INVERSION = false;
		public static final boolean OUTAKE_MOTOR_INVERSION = true;

		public static final int EJECT_TIME_SECS = 5;
		public static final int INTAKE_TIME_SECS = 4;

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
			public static final int INTAKE_BUTTON = Axis.kRightTrigger.value;
			public static final int SHOOTER_BUTTON = Axis.kRightTrigger.value;
			public static final int EJECT_BUTTON = Button.kA.value;
			public static final int AMP_BUTTON = Button.kLeftBumper.value;
			public static final int BOOLEAN_BUTTON = Button.kRightBumper.value;

		}
		public static final double JOY_THRESH = 0.01;
        public static final double MIN_AXIS_TRIGGER_VALUE = 0.25;
	}
}
