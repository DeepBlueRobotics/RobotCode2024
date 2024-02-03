// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

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
		// Array Indexes (Just to make things easier to read)
        public static final int ARM = 0;
		//Motor port
		public static final int MOTOR_PORT1 = 7;
		public final static int MOTOR_PORT2 = 8;
		//all angles in rot here
		public static final double intakeAngle = -.1;
		public static final double ampAngle = .3;
		public static final double speakerAngle = .4;
		//if needed
		public static final double UPPER_ANGLE = Math.toRadians(70); 
		public static final double LOWER_ANGLE = Math.toRadians(0);
		public static final double ARM_DICONT_RAD = (LOWER_ANGLE + UPPER_ANGLE) /2 - Math.PI;
		// Feedforward
		public static final double kS = 0.1;
		public static final double kG = 0.1;
		public static final double kV = 0.1;
		public static final double kA = 0.1;
		// PID Constants
		// placeholder numbers for now
        public static double[] kP = {0.1, 0.1};
        public static double[] kI = {0.1, 0.1};
        public static double[] kD = {0.1, 0.1};
		
		//Arm measurements - ALL OF THEM ARE PLACEHOLDERS THE NUMBERS MEAN NOTHING
		public static final double COM_ARM_LENGTH_METERS = 1;
		public static final double ARM_MASS_KG = 1;

		public static final double MAX_FF_VEL = 1; // rot / s
		public static final double MAX_FF_ACCEL = 1; // rot / s^2
		public static TrapezoidProfile.Constraints trapConstraints = new TrapezoidProfile.Constraints(MAX_FF_VEL, MAX_FF_ACCEL);
		
	}
	public static final class IntakeShooter {
		//in set() speed
		public static final double idleSpeed = 0;
		public static final double intakeSpeed = .7;
		public static final double outtakeSpeed = 1;

	}
	
	public static final class OI {
        public static final class Driver {
            public static final int port = 0;
        }
        public static final class Manipulator {
            public static final int port = 1;
        }
    }
}
