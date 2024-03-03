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

	

		//Motor port
		public static final int ARM_MOTOR_PORT_MASTER = 7;
		public final static int ARM_MOTOR_PORT_FOLLOWER = 8;
		//Config for motors
		public static final boolean MOTOR_INVERTED_MASTER = true; //Todo: find all these (they are definetely wrong)
		public static final boolean MOTOR_INVERTED_FOLLOWER = false;
		public static final double ROTATION_TO_RAD = 2 * Math.PI;
		public static final boolean ENCODER_INVERTED = false;
		public static final double ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD = 0; //placeholder
		public static final double POS_TOLERANCE_RAD = 0; //placeholder
		public static final double VEL_TOLERANCE_RAD_P_SEC = 0; //placeholder
		public static final int MAX_VOLTAGE = 12;
		public static final double ENCODER_OFFSET_RAD = 0;
		
		//TODO: finish understand why this is broken public static final Measure<Angle> INTAKE_ANGLE = Degrees.to(-1);

		// USE RADIANS FOR THE ARM
		public static final double INTAKE_ANGLE = Units.degreesToRadians(0);
		public static final double AMP_ANGLE = Units.degreesToRadians(105);
		public static final double SUBWOFFER_ANGLE = Units.degreesToRadians(24);
		public static final double SAFE_ZONE_ANGLE = Units.degreesToRadians(24);
		public static final double PODIUM_ANGLE = Units.degreesToRadians(24);
		public static final double CLIMBER_UP_ANGLE = Units.degreesToRadians(24);
		public static final double CLIMBER_DOWN_ANGLE = Units.degreesToRadians(24);


		//PID, Feedforward, Trapezoid
		public static final double kP = 0.1;
		public static final double kI = 0.1;
		public static final double kD = 0.1;
		public static final double kS = 0.1;
		public static final double kG = 0.1;
		public static final double kV = 0.1;
		public static final double kA = 0.1;
		public static final double IZONE = 4;
		public static final double MAX_FF_VEL = 1; // rot / s
		public static final double MAX_FF_ACCEL = 1; // rot / s^2 

		//if needed
		public static final double COM_ARM_LENGTH_METERS = 0.381 ;
		public static final double ARM_MASS_KG = 9.59302503;
		public static final double UPPER_ANGLE_LIMIT = Units.degreesToRadians(70); 
		public static final double LOWER_ANGLE_LIMIT = Units.degreesToRadians(0);
		public static final double ARM_DISCONT_RAD = (LOWER_ANGLE_LIMIT + UPPER_ANGLE_LIMIT) /2 - Math.PI;
		public static TrapezoidProfile.Constraints TRAP_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_FF_VEL, MAX_FF_ACCEL);
		//other0;

		public static final double MARGIN_OF_ERROR = Math.PI/18;
		public static final double ARM_LOWER_LIMIT_RAD = -3.569 + MARGIN_OF_ERROR;
		public static final double ARM_UPPER_LIMIT_RAD = .36 - MARGIN_OF_ERROR;
		public static final double ARM_DISCONTINUITY_RAD = (ARM_LOWER_LIMIT_RAD + ARM_UPPER_LIMIT_RAD) / 2 - Math.PI;
		//Arm buttons
		
	}
	
	public static final class IntakeShooter {
		//in set() speed
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
			public static final int RAISE_TO_GROUND_BUTTON= Button.kStart.value;
			public static final int RAISE_TO_CLIMBER_BUTTON = Button.kLeftBumper.value;
			public static final int LOWER_TO_CLIMBER_BUTTON = Button.kRightBumper.value;
        }
    }
}
