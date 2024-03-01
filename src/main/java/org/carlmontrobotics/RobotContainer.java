// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;
//199 files
// import org.carlmontrobotics.subsystems.*;
import org.carlmontrobotics.commands.*;
import static org.carlmontrobotics.Constants.OI;

import org.carlmontrobotics.Constants.OI;
import org.carlmontrobotics.commands.ArmTeleop;
import org.carlmontrobotics.commands.armAmpPos;
import org.carlmontrobotics.commands.armGroundPos;
import org.carlmontrobotics.commands.armPodiumSpeakerPos;
import org.carlmontrobotics.subsystems.Arm;

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
    new JoystickButton(manipulatorController, Constants.Arm.raiseToSpeakerPodButton).onTrue(new InstantCommand(() -> {arm.cancelArmCommand(); new armPodiumSpeakerPos(arm);}));
    new JoystickButton(manipulatorController, Constants.Arm.raiseToSpeakerNextButton).onTrue(new InstantCommand(() -> {arm.cancelArmCommand(); new armNextToSpeakerPos(arm);}));
    new JoystickButton(manipulatorController, Constants.Arm.raiseToSpeakerSafeButton).onTrue(new InstantCommand(() -> {arm.cancelArmCommand(); new armSafeZonePos(arm);}));
    // Amp and Intake Buttons
    new JoystickButton(manipulatorController, Constants.Arm.raiseToAmpButton).onTrue(new InstantCommand(() -> {arm.cancelArmCommand(); new armAmpPos(arm);}));
    new JoystickButton(manipulatorController, Constants.Arm.raiseToGroundButton).onTrue(new InstantCommand(() -> {arm.cancelArmCommand(); new armGroundPos(arm);}));
    // Cimber Buttons
    new JoystickButton(manipulatorController, Constants.Arm.raiseToClimberButton).onTrue(new InstantCommand(() -> {arm.cancelArmCommand(); new armClimberUpPos(arm);}));
    new JoystickButton(manipulatorController, Constants.Arm.lowerToClimberButton).onTrue(new InstantCommand(() -> {arm.cancelArmCommand(); new armClimberDownPos(arm);}));
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
