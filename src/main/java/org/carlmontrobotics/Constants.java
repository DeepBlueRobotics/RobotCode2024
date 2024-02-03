// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units.*;

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
		public static final int LEFT_MOTOR_PORT = 7;
		public final static int RIGHT_MOTOR_PORT = 8;
		//all angles in rot here
		//TODO: finish understand why this is broken public static final Measure<Angle> INTAKE_ANGLE = Degrees.to(-1);
		public static final double intakeAngle = Math.toDegrees(-0.1);
		public static final double ampAngle = Math.toDegrees(0.3);
		public static final double speakerAngle = Math.toDegrees(0.4);
		//if needed
		public static final double UPPER_ANGLE_LIMIT = Math.toRadians(70); 
		public static final double LOWER_ANGLE_LIMIT = Math.toRadians(0);
		public static final double ARM_DICONT_RAD = (LOWER_ANGLE_LIMIT + UPPER_ANGLE_LIMIT) /2 - Math.PI;
		//Motor Controllers: pid, FF
		public static final double[] pidVals = new double[] { 0.1, 0.0, 0.1 };
		// Feedforward
		public static final double kS = 0.1;
		public static final double kG = 0.1;
		public static final double kV = 0.1;
		public static final double kA = 0.1;
		// PID Constants
		// placeholder numbers for now
        public static final double kP = 0.1;
		public static final double kI = 0.1;
		public static final double kD = 0.1;
		
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
