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
import static org.carlmontrobotics.Constants.IntakeShoot.*;

import java.util.function.BooleanSupplier;

import org.carlmontrobotics.Constants.OI;
import org.carlmontrobotics.Constants.OI.Manipulator;
//subsystems
//import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.IntakeShooter;

import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.geometry.Rotation2d;
//controllers
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;

//commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.carlmontrobotics.commands.*;

//control bindings
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  IntakeShooter intakeShooter = new IntakeShooter();
  // private final Drivetrain drivetrain = new Drivetrain();

  // 1. using GenericHID allows us to use different kinds of controllers
  // 2. Use absolute paths from constants to reduce confusion
  public final GenericHID driverController = new GenericHID(OI.Driver.port);
  public final GenericHID manipulatorController = new GenericHID(OI.Manipulator.port);

  //Drivetrain drivetrain = new Drivetrain();
  //Limelight limelight = new Limelight(drivetrain);

  private final String[] autoNames = new String[] {/*These are assumed to be equal to the file names*/
    "Penis"
  };

  public RobotContainer() {
		//defaultCommands: elevator, dt
		//(pass in controller!)

    // setupAutos();
		setDefaultCommands();
		setBindingsDriver();
		setBindingsManipulator();
  }
  
	private void setDefaultCommands() {
    // drivetrain.setDefaultCommand(new TeleopDrive(
    //   drivetrain,
    //   (DoubleSupplier) () -> ProcessedAxisValue(driverController, Axis.kLeftY),
    //   (DoubleSupplier) () -> ProcessedAxisValue(driverController, Axis.kLeftX),
    //   (DoubleSupplier) () -> ProcessedAxisValue(driverController, Axis.kRightX),
    //   (BooleanSupplier)() -> driverController.getRawButton(OI.Driver.slowDriveButton)
    // ));
  }
  private void setBindingsDriver() {
		// reset field orientation??
    // new JoystickButton(driverController, Driver.resetFieldOrientationButton).onTrue(
    //   new InstantCommand(drivetrain::resetFieldOrientation));
    // // toggle orientation plane between field and relative
    // new JoystickButton(driverController, Driver.toggleFieldOrientedButton).onTrue(
    //   new InstantCommand(() -> drivetrain.setFieldOriented(!drivetrain.getFieldOriented())));
    
      // 4 cardinal directions on arrowpad
    // new JoystickButton(driverController, Driver.rotateFieldRelative0Deg).onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(0), drivetrain));
    // new JoystickButton(driverController, Driver.rotateFieldRelative90Deg).onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(-90), drivetrain));
    // new JoystickButton(driverController, Driver.rotateFieldRelative180Deg).onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(180), drivetrain));
    // new JoystickButton(driverController, Driver.rotateFieldRelative270Deg).onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(90), drivetrain));
    
    //TODO: 3 cardinal directions on letterpad
    // new JoystickButton(driverController, Driver.rotateFieldRelative240Deg).onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(90), drivetrain));
    // new JoystickButton(driverController, Driver.rotateFieldRelative120Deg).onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(90), drivetrain));
    // new JoystickButton(driverController, Driver.rotateFieldRelative240Deg).onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(90), drivetrain));
	}

  private void setBindingsManipulator() {
  //have the trigger and button bindings here call the Intake, Shoot, and Eject commands

  //NEW BINDINGS(easier for manipulator)
  //Xbox right trigger axis -> Shoot
  //Xbox left trigger axis -> Intake
  //Xbox right bumper button -> Speaker (arm position)
  //Xbox left bumper button -> Amp (arm position and RPM)
  //Xbox X button -> Intake(arm position)
  //Xbox A button -> Eject


    //COMBINING BINDINGS WITH ARM
    /*/Eject/*/
    new JoystickButton(manipulatorController, OI.Manipulator.EjectButton).onTrue(new EjectRPM(intakeShooter));
    
    /*/Amp Shooting/*/      
    //TODO: add the three buttons for the 3 spots we want to shoot from (safe zone, podium, subwoffer)
    new JoystickButton(manipulatorController, OI.Manipulator.AmpButton).onTrue(new SequentialCommandGroup(
      new ShooterToRPM(intakeShooter, AMP_RPM),
      new PassToOutake(intakeShooter)
    ));
    
    /*/Shooting/*/
    new JoystickButton(manipulatorController, OI.Manipulator.ShooterButton.value).onTrue(new SequentialCommandGroup(
      new ShooterToRPM(intakeShooter, SPEAKER_RPM),
      new PassToOutake(intakeShooter)
    ));

    /*/Intake/*/ 
    new JoystickButton(manipulatorController, OI.Manipulator.IntakeButton.value).onTrue(new Intake(intakeShooter)); //I don't know the UI so this is placeholder
  
  }



  
  public Command getAutonomousCommand() {
    Command autoCommand = null;


    //return autoPath == null ? new PrintCommand("No Autonomous Routine selected") : autoCommand;
     return new PrintCommand("No Auto, this is drivetrain branch");
	}

	/**
	 * Flips an axis' Y coordinates upside down, but only if the select axis is a joystick axis
	 *
	 * @param hid The controller/plane joystick the axis is on
	 * @param axis The processed axis
	 * @return The processed value.
	 */
	private double getStickValue(GenericHID hid, Axis axis) {
		return hid.getRawAxis(axis.value) * (axis == Axis.kLeftY || axis == Axis.kRightY ? -1 : 1);
	}
	/**
	 * Processes an input from the joystick into a value between -1 and 1, sinusoidally instead of linearly
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
	 * Combines both getStickValue and inputProcessing into a single function for processing joystick outputs
	 *
	 * @param hid The controller/plane joystick the axis is on
	 * @param axis The processed axis
	 * @return The processed value.
	 */
	private double ProcessedAxisValue(GenericHID hid, Axis axis){
		return inputProcessing(getStickValue(hid, axis));
  }

  private Trigger axisTrigger(GenericHID manipulatorController, Axis krighttrigger) {
    return axisTrigger(manipulatorController, krighttrigger);
  }
  private Trigger axisTrigger2(GenericHID manipulatorController, Axis klefttrigger) {
    return axisTrigger(manipulatorController, klefttrigger);
  }

}
