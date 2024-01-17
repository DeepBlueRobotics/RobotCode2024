// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

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
		//Motor port
		public final static int MOTOR_PORT = 8;
		//all angles in rot here
		public static final double intakeAngle = -.1;
		public static final double ampAngle = .3;
		public static final double speakerAngle = .4;
		//if needed
//		public static final trapAngle = 80;
		
		//Motor Controllers: pid, FF
		public static final double[] pidVals = new double[] { 0.1, 0.0, 0.1 };
		// Feedforward
		public static final double kS = 0.1;
		public static final double kG = 0.1;
		public static final double kV = 0.1;
		public static final double kA = 0.1;
		
		public static final double MAX_VEL = 1; // rot / s
		public static final double MAX_ACCEL = 1; // rot / s^2
		public static TrapezoidProfile.Constraints trapConstraints = new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL);
		
	}
	public static final class IntakeShooter {
		//in set() speed
		public static final double idleSpeed = 0;
		public static final double intakeSpeed = .7;
		public static final double outtakeSpeed = 1;

	}
	//array or enum with arm and wrist values 

}
