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
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final class Drivetrain {
	}
	
	public static final class Arm {
		//#region subsytem constants
		
		// ports
		public static final int MASTER_ARM_MOTOR = 7;
		public static final int FOLLOW_ARM_MOTOR = 8;

		// config
		public static final int MASTER = 0;
		public static final int FOLLOWER = 1;
		public static final boolean[] motorInverted = {true, true}; //Todo: find all these (they are definetely wrong)
		public static final boolean encoderInverted = false;
		// TODO: Figure out offset
		public static final double ENCODER_OFFSET = 0;
		
		// goal positions
		public static final double INTAKE_ANGLE = Units.degreesToRadians(0);
		public static final double AMP_ANGLE = Units.degreesToRadians(103);
		public static final double SPEAKER_ANGLE = Units.degreesToRadians(24);
		public static final double CLIMB_ANGLE = Units.degreesToRadians(24);
		
		// Feedforward
		public static final double kG = 0;
		public static final double kS = 0;
		public static final double kV = 0;
		public static final double kA = 0;

		// PID Constants
        public static final double kP = 0;
		public static final double kI = 0;
		public static final double kD = 0;

		// Trapezoid Profile and others
		public static final int MAX_VOLTAGE = 12;
		public static final double rotationToRad = 2 * Math.PI;
		public static final double MAX_FF_VEL = 1; // rot / s
		public static final double MAX_FF_ACCEL = 1; // rot / s^2 
		//I assume to max vel and accel are in meters per second
		public static TrapezoidProfile.Constraints armConstraints = new TrapezoidProfile.Constraints(MAX_FF_VEL, MAX_FF_ACCEL);

		// Bounds
		public static final double ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD = 0;
		public static final double MARGIN_OF_ERROR = Math.PI/18;
		public static double posToleranceRad = .07;
		public static double velToleranceRadPSec = 1;
		// TODO: Determine actual values
		public static final double ARM_LOWER_LIMIT_RAD = -3.569 + MARGIN_OF_ERROR;
		public static final double ARM_UPPER_LIMIT_RAD = .36 - MARGIN_OF_ERROR;
		public static final double ARM_DISCONTINUITY_RAD = (ARM_LOWER_LIMIT_RAD + ARM_UPPER_LIMIT_RAD) / 2 - Math.PI;

		//#endregion
	}
	
	public static final class OI {
		public static final double JOY_THRESH = 0.01;
        public static final class Driver {
            public static final int port = 0;
        }
        public static final class Manipulator {
            public static final int port = 1;

			// Arm commands
			public static final int raiseToSpeakerPodButton = Button.kY.value;
			public static final int raiseToAmpButton = Button.kB.value;
			public static final int raiseToSpeakerSafeButton = Button.kA.value;
			public static final int raiseToSpeakerNextButton = Button.kX.value;
			public static final int raiseToGroundButton = Button.kStart.value;
			public static final int raiseToClimberButton = Button.kLeftBumper.value;
			public static final int lowerToClimberButton = Button.kRightBumper.value;
        }
    }
}
