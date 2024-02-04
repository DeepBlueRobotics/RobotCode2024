// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

import org.carlmontrobotics.Constants.OI;
import org.carlmontrobotics.Constants.OI.Driver;
import org.carlmontrobotics.Constants.OI.Manipulator;
//199 files
import org.carlmontrobotics.subsystems.*;
import org.carlmontrobotics.commands.*;
import static org.carlmontrobotics.Constants.OI;

import edu.wpi.first.math.geometry.Rotation2d;
//controllers
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController.Axis;

//commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

//control bindings
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

//pathplanner
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import java.util.ArrayList;
import java.util.function.Supplier;
import java.util.stream.Collectors;

//java
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

public class RobotContainer {
	// set up subsystems / controllers / limelight
	public final GenericHID driverController = new GenericHID(OI.Driver.port);
	public final GenericHID manipulatorController = new GenericHID(OI.Manipulator.port);

	Drivetrain drivetrain = new Drivetrain();

	private final String[] autoNames = new String[] { /* These are assumed to be equal to the file names */
			"Penis"
	};

	public RobotContainer() {
		// defaultCommands: elevator, dt
		// (pass in controller!)

		// setupAutos();
		setDefaultCommands();
		setBindingsDriver();
		setBindingsManipulator();
	}

	private void setDefaultCommands() {
		drivetrain.setDefaultCommand(new TeleopDrive(
			drivetrain,
			(DoubleSupplier) () -> ProcessedAxisValue(driverController, Axis.kLeftY),
			(DoubleSupplier) () -> ProcessedAxisValue(driverController, Axis.kLeftX),
			(DoubleSupplier) () -> ProcessedAxisValue(driverController, Axis.kRightX),
			(BooleanSupplier) () -> driverController.getRawButton(OI.Driver.slowDriveButton)));
	}

	private void setBindingsDriver() {
		// reset field orientation??
		new JoystickButton(driverController, Driver.resetFieldOrientationButton).onTrue(
				new InstantCommand(drivetrain::resetFieldOrientation));
		// toggle orientation plane between field and relative
		new JoystickButton(driverController, Driver.toggleFieldOrientedButton).onTrue(
				new InstantCommand(() -> drivetrain.setFieldOriented(!drivetrain.getFieldOriented())));

		new JoystickButton(driverController, Driver.quasistaticForward)
			.whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		new JoystickButton(driverController, Driver.quasistaticBackward)
			.whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		new JoystickButton(driverController, Driver.dynamicForward)
			.whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
		new JoystickButton(driverController, Driver.dynamicBackward)
			.whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

		// 4 cardinal directions on arrowpad
		// new JoystickButton(driverController, Driver.rotateFieldRelative0Deg)
		// 		.onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(0), drivetrain));
		// new JoystickButton(driverController, Driver.rotateFieldRelative90Deg)
		// 		.onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(-90), drivetrain));
		// new JoystickButton(driverController, Driver.rotateFieldRelative180Deg)
		// 		.onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(180), drivetrain));
		// new JoystickButton(driverController, Driver.rotateFieldRelative270Deg)
		// 		.onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(90), drivetrain));

		// TODO: 3 cardinal directions on letterpad
		// new JoystickButton(driverController,
		// Driver.rotateFieldRelative240Deg).onTrue(new
		// RotateToFieldRelativeAngle(Rotation2d.fromDegrees(90), drivetrain));
		// new JoystickButton(driverController,
		// Driver.rotateFieldRelative120Deg).onTrue(new
		// RotateToFieldRelativeAngle(Rotation2d.fromDegrees(90), drivetrain));
		// new JoystickButton(driverController,
		// Driver.rotateFieldRelative240Deg).onTrue(new
		// RotateToFieldRelativeAngle(Rotation2d.fromDegrees(90), drivetrain));
	}

	private void setBindingsManipulator() {
		// 3 setpositions of elevator on arrowpad
		// intake/outtake on triggers

		// 3 setpositions of arm on letterpad
		// right joystick used for manual arm control
	}

	public Command getAutonomousCommand() {
		Command autoCommand = null;

		// return autoPath == null ? new PrintCommand("No Autonomous Routine selected")
		// : autoCommand;
		return new PrintCommand("No Auto, this is drivetrain branch");
	}

	/**
	 * Flips an axis' Y coordinates upside down, but only if the select axis is a
	 * joystick axis
	 *
	 * @param hid  The controller/plane joystick the axis is on
	 * @param axis The processed axis
	 * @return The processed value.
	 */
	private double getStickValue(GenericHID hid, Axis axis) {
		return hid.getRawAxis(axis.value) * (axis == Axis.kLeftY || axis == Axis.kRightY ? -1 : 1);
	}

	/**
	 * Processes an input from the joystick into a value between -1 and 1,
	 * sinusoidally instead of linearly
	 *
	 * @param value The value to be processed.
	 * @return The processed value.
	 */
	private double inputProcessing(double value) {
		double processedInput;
		// processedInput =
		// (((1-Math.cos(value*Math.PI))/2)*((1-Math.cos(value*Math.PI))/2))*(value/Math.abs(value));
		processedInput = Math.copySign(((1 - Math.cos(value * Math.PI)) / 2) * ((1 - Math.cos(value * Math.PI)) / 2),
				value);
		return processedInput;
	}

	/**
	 * Combines both getStickValue and inputProcessing into a single function for
	 * processing joystick outputs
	 *
	 * @param hid  The controller/plane joystick the axis is on
	 * @param axis The processed axis
	 * @return The processed value.
	 */
	private double ProcessedAxisValue(GenericHID hid, Axis axis) {
		return inputProcessing(getStickValue(hid, axis));
	}
}
