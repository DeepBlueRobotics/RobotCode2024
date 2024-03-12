// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

import static org.carlmontrobotics.Constants.OI.Manipulator.AMP_BUTTON;
import static org.carlmontrobotics.Constants.OI.Manipulator.EJECT_BUTTON;

import java.util.function.BooleanSupplier;
//java
import java.util.function.DoubleSupplier;

import org.carlmontrobotics.Constants.OI;
import org.carlmontrobotics.Constants.OI.Driver;
import org.carlmontrobotics.Constants.OI.Manipulator;
//199 files
import org.carlmontrobotics.commands.AmpRPM;
import org.carlmontrobotics.commands.Eject;
import org.carlmontrobotics.commands.Intake;
import org.carlmontrobotics.commands.PassToOutake;
import org.carlmontrobotics.commands.TeleopDrive;
import org.carlmontrobotics.subsystems.Drivetrain;
//subsystems
//import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
//commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
//control bindings
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {
	// set up subsystems / controllers / limelight
	public final GenericHID driverController = new GenericHID(OI.Driver.port);
	public final GenericHID manipulatorController = new GenericHID(OI.Manipulator.port);

	Drivetrain drivetrain = new Drivetrain();
	private final IntakeShooter intakeShooter = new IntakeShooter();

	private final String[] autoNames = new String[] { /* These are assumed to be equal to the file names */
			"nothing here"
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
		// // toggle orientation plane between field and relative
		// new JoystickButton(driverController,
		// Driver.toggleFieldOrientedButton).onTrue(
		// new InstantCommand(() ->
		// drivetrain.setFieldOriented(!drivetrain.getFieldOriented())));

		new JoystickButton(driverController, Button.kY.value)
				.whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		new JoystickButton(driverController, Button.kA.value)
				.whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		new JoystickButton(driverController, Button.kB.value)
				.whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
		new JoystickButton(driverController, Button.kX.value)
				.whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

		// 4 cardinal directions on arrowpad
		// new JoystickButton(driverController, Driver.rotateFieldRelative0Deg)
		// .onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(0),
		// drivetrain));
		// new JoystickButton(driverController, Driver.rotateFieldRelative90Deg)
		// .onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(-90),
		// drivetrain));
		// new JoystickButton(driverController, Driver.rotateFieldRelative180Deg)
		// .onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(180),
		// drivetrain));
		// new JoystickButton(driverController, Driver.rotateFieldRelative270Deg)
		// .onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(90),
		// drivetrain));

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

		// have the trigger and button bindings here call the Intake, Shoot, and Eject
		// commands
		// NEW BINDINGS(easier for manipulator)
		// Xbox right trigger axis -> Shoot
		// Xbox left trigger axis -> Intake
		// Xbox right bumper button -> Speaker (arm position)
		// Xbox left bumper button -> Amp (arm position and RPM and Eject)
		// Xbox X button -> Intake(arm position)

		/* /Eject also for AMP/ */
		new JoystickButton(manipulatorController, EJECT_BUTTON).onTrue(new Eject(intakeShooter));
		new JoystickButton(manipulatorController, EJECT_BUTTON).onFalse(new InstantCommand());

		new JoystickButton(manipulatorController, AMP_BUTTON).onTrue(new AmpRPM(intakeShooter));
		new JoystickButton(manipulatorController, AMP_BUTTON).onFalse(new InstantCommand());

		/* /Shooting/ */
		// new JoystickButton(manipulatorController, SHOOTER_BUTTON).onTrue(new
		// PassToOutake(intakeShooter));

		/* /Intake/ */
		// new JoystickButton(manipulatorController, INTAKE_BUTTON).onTrue(new
		// Intake(intakeShooter)); // I don't know the UI
		// so this is
		// placeholder

		axisTrigger(manipulatorController, Manipulator.SHOOTER_BUTTON)
				.onTrue(
						new PassToOutake(intakeShooter));
		axisTrigger(manipulatorController, Manipulator.SHOOTER_BUTTON)
				.onFalse(
						new InstantCommand(intakeShooter::stopOutake, intakeShooter));

		axisTrigger(manipulatorController, Manipulator.INTAKE_BUTTON)
				.onTrue(
						new Intake(intakeShooter));

		axisTrigger(manipulatorController, Manipulator.INTAKE_BUTTON)
				.onTrue(
						new Intake(intakeShooter));

		// TODO: ask charles if passing in controller is okay
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

	private final Trigger axisTrigger(GenericHID stick, Axis axis) {
		return new Trigger(() -> Math.abs(getStickValue(stick, axis)) > OI.MIN_AXIS_TRIGGER_VALUE);
	}

}
