// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
	public static final class Drivetrain {
	}
	public static final class Arm{}

	public static final class IntakeShoot {
		// PID values

		public static final double[] kP = { 0.0001, 0 };
		public static final double kI = 0;
		public static final double[] kD = { 0, 0 };
		public static final double kS = 0.29753;
		public static final double kV = 0.077913;
		public static final double kA = 0.05289;
		public static final int distanceSensorPort1 = 10; //port
		public static final int distanceSensorPort2 = 0; //port
		public static final double distanceBetweenSensors = 8.189; // inches
		public static final double OFFSETFROMGROUND = 21; // in
		public static final double dsDepth = 9.97;
		public static final double detectDistance = 13;


		public static final double INTAKE_RPM = -6000;
		public static final double PASS_RPM = 300;

		public static final double AMP_RPM = 1500;
		public static final double SPEAKER_RPM = 6000;

		public static final double RPM_TOLERANCE = 10;
		public static final double SpeakerHeight = 21; // inches 

	}

	public static final class OI {
		public static final class Driver {
			public static final int port = 0;
		}

		public static final class Manipulator {
			public static final int port = 1;
			public static final Axis IntakeButton = Axis.kRightTrigger;
			public static final Axis ShooterButton = Axis.kRightTrigger;
			public static final int EjectButton = Button.kA.value;
			public static final int AmpButton = Button.kLeftBumper.value;
			public static final int BooleanButton = Button.kRightBumper.value;

		}
	}
}
