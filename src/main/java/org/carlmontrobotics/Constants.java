// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController.Axis;
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
	public static final class Drivetrain {}
	
	public static final class Arm {
		
		//all angles in rot here
		public static final double intakeAngle = -.1;
		public static final double ampAngle = .3;
		public static final double speakerAngle = .4;
		//if needed
		//public static final trapAngle = 80;
		
		//Motor Controllers: pid, FF
		//PID
		public static final double[] pidVals = new double[] { 0.1, 0.0, 0.1 };
		//Feed Forward
		public static final double[] FeedforwardVals = new double[] { /*/kS/*/0.1, /*/kG/*/0.1, /*/kV/*/0.1, /*/kA/*/0.1 };
	
		
		public static final double MAX_FF_VEL = 1; // rot / s
		public static final double MAX_FF_ACCEL = 1; // rot / s^2
		public static TrapezoidProfile.Constraints trapConstraints = new TrapezoidProfile.Constraints(MAX_FF_VEL, MAX_FF_ACCEL);
		
	}
	public static final class IntakeShooter {
		//PID values
		public final double[] shooterPID = new double[] {.1,0,0};
		//PID/FeedForward values
		public final static double kS = 1.0851;
            public static final double kV = 0.066742;
            public static final double kA = 0.0076259;
            
            public static final int dsPort1 = 10;
            public static final int dsPort2 = 0;

		public static final double idleSpeed = 0;
		public static final double intakeSpeed = .7;

		public static final double ampRPM = 1;
		public static final double speakerRPM = 1;
		//2 different rpms for 2 arm scoring positions (armAmpPos, armSpeakerPos)
		//change these values as needed later
		public static final double intakeRPM = 4000;
		public static final int OuttakePort = 17;
		public static final int IntakePort = 1;
		public static final double kP = 0.00014312;
    	public static final double kD = 0;
    	public static final double kI = 0;
		public static final int beamBreakPort = 11;
		public static final int distSensorPort = 12;
	}

	public static class ManipulatorButtons{
		public static final Axis IntakeButton = Axis.kRightTrigger;
		public static final Axis ShooterButton = Axis.kRightTrigger;
		public static final int EjectButton = Button.kA.value;
		public static final int AmpButton = Button.kRightBumper.value;


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
