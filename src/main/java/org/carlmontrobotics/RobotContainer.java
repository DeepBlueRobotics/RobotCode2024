// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;
//199 files
// import org.carlmontrobotics.subsystems.*;
import org.carlmontrobotics.commands.*;
import static org.carlmontrobotics.Constants.OI;
import static org.carlmontrobotics.Constants.Arm.*;

import org.carlmontrobotics.Constants.OI;
import org.carlmontrobotics.subsystems.Arm;

import org.carlmontrobotics.Constants;

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

public class RobotContainer {
  //defaultCommands: elevator, dt
		//(pass in controller!)

  //set up subsystems / controllers / limelight
  //1. using GenericHID allows us to use different kinds of controllers
  //2. Use absolute paths from constants to reduce confusion
  public final GenericHID driverController = new GenericHID(OI.Driver.port);
  public final GenericHID manipulatorController = new GenericHID(OI.Manipulator.port);

	public Arm arm = new Arm();

  public RobotContainer() {

    setDefaultCommands();
    setBindingsDriver();
    setBindingsManipulator();
  }

  private void setDefaultCommands() {

    // drivetrain.setDefaultCommand(new TeleopDrive(
    //   drivetrain,
    //   () -> ProcessedAxisValue(driverController, Axis.kLeftY)),
    //   () -> ProcessedAxisValue(driverController, Axis.kLeftX)),
    //   () -> ProcessedAxisValue(driverController, Axis.kRightX)),
    //   () -> driverController.getRawButton(OI.Driver.slowDriveButton)
    // ));

    arm.setDefaultCommand(new ArmTeleop(arm, () -> inputProcessing(getStickValue(manipulatorController, Axis.kLeftY))));
  }
  private void setBindingsDriver() {
    // 4 cardinal directions on arrowpad
		// slowmode toggle on trigger
		// 3 cardinal directions on letterpad
  }
  private void setBindingsManipulator() {
		// intake/outtake on triggers
    //Left trigger will be intake whilst right trigger will be outtake
    //3 setpositions of arm on letterpad
    //Up is Speaker, down is ground, right is Amp
    //right joystick used for manual arm control

    // Speaker Buttons
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.RAISE_TO_SPEAKER_POD_BUTTON).onTrue(new InstantCommand(() -> {arm.driveArm(Constants.Arm.PODIUM_ANGLE);}));
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.RAISE_TO_SPEAKER_NEXT_BUTTON).onTrue(new InstantCommand(() -> {arm.driveArm(Constants.Arm.SUBWOFFER_ANGLE);}));
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.RAISE_TO_SPEAKER_SAFE_BUTTON).onTrue(new InstantCommand(() -> {arm.driveArm(Constants.Arm.SAFE_ZONE_ANGLE);}));
    // Amp and Intake Buttons
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.RAISE_TO_AMP_BUTTON).onTrue(new InstantCommand(() -> {arm.driveArm(Constants.Arm.AMP_ANGLE);}));
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.RAISE_TO_GROUND_BUTTON).onTrue(new InstantCommand(() -> {arm.driveArm(Constants.Arm.INTAKE_ANGLE);}));
    // Cimber Buttons
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.RAISE_TO_CLIMBER_BUTTON).onTrue(new InstantCommand(() -> {arm.driveArm(Constants.Arm.CLIMBER_UP_ANGLE);}));
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.LOWER_TO_CLIMBER_BUTTON).onTrue(new InstantCommand(() -> {arm.driveArm(Constants.Arm.CLIMBER_DOWN_ANGLE);}));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
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
}
