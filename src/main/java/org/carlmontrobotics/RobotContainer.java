// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

import org.carlmontrobotics.Constants.OI;
//199 files
// import org.carlmontrobotics.subsystems.*;
import org.carlmontrobotics.commands.*;
import static org.carlmontrobotics.Constants.OI;
import static org.carlmontrobotics.Constants.Arm.*;
import static org.carlmontrobotics.Constants.OI.Manipulator.*;
import org.carlmontrobotics.subsystems.Arm;
import org.carlmontrobotics.Constants;

//subsystems
//import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.IntakeShooter;

//controllers
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;

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

public class RobotContainer {
  // defaultCommands: elevator, dt
  // (pass in controller!)

  // set up subsystems / controllers / limelight
  // 1. using GenericHID allows us to use different kinds of controllers
  // 2. Use absolute paths from constants to reduce confusion
  public final GenericHID driverController = new GenericHID(OI.Driver.port);
  public final GenericHID manipulatorController = new GenericHID(OI.Manipulator.port);

  public Arm arm = new Arm();
  // private final Drivetrain drivetrain = new Drivetrain();

  // 1. using GenericHID allows us to use different kinds of controllers
  // 2. Use absolute paths from constants to reduce confusion
  //private final GenericHID driverController = new GenericHID(OI.Driver.port);
  //private final GenericHID manipulatorController = new GenericHID(OI.Manipulator.port);
  private final IntakeShooter intakeShooter = new IntakeShooter();
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

    arm.setDefaultCommand(new ArmTeleop(arm, 
      () -> DeadzonedAxis(inputProcessing(getStickValue(manipulatorController, Axis.kLeftY)))));
  }
  private void setBindingsDriver() {
    new JoystickButton(driverController, Button.kX.value)
        .whileTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    new JoystickButton(driverController, Button.kY.value)
        .whileTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    new JoystickButton(driverController, Button.kB.value).whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kForward));
    new JoystickButton(driverController, Button.kA.value).whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // 4 cardinal directions on arrowpad
    // slowmode toggle on trigger
    // 3 cardinal directions on letterpad
  }
  private void setBindingsManipulator() {
    // intake/outtake on triggers
    // Left trigger will be intake whilst right trigger will be outtake
    // 3 setpositions of arm on letterpad
    // Up is Speaker, down is ground, right is Amp
    // right joystick used for manual arm control
    // COMMENT THESE OUT DURING SYSID TESTING
    // Speaker Buttons
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.RAISE_TO_SPEAKER_POD_BUTTON)
        .onTrue(new InstantCommand(() -> {
          arm.setArmTarget(PODIUM_ANGLE_RAD);
        }));
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.RAISE_TO_SPEAKER_NEXT_BUTTON)
        .onTrue(new InstantCommand(() -> {
          arm.setArmTarget(SUBWOFFER_ANGLE_RAD);
        }));
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.RAISE_TO_SPEAKER_SAFE_BUTTON)
        .onTrue(new InstantCommand(() -> {
          arm.setArmTarget(SAFE_ZONE_ANGLE_RAD);
        }));
    // Amp and Intake Buttons
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.RAISE_TO_AMP_BUTTON)
        .onTrue(new InstantCommand(() -> {
          arm.setArmTarget(AMP_ANGLE_RAD);
        }));
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.RAISE_TO_GROUND_BUTTON)
        .onTrue(new InstantCommand(() -> {
          arm.setArmTarget(INTAKE_ANGLE_RAD);
        }));
    // Cimber Buttons
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.RAISE_TO_CLIMBER_BUTTON)
        .onTrue(new InstantCommand(() -> {
          arm.setArmTarget(CLIMBER_UP_ANGLE_RAD);
        }));
    new JoystickButton(manipulatorController, Constants.OI.Manipulator.LOWER_TO_CLIMBER_BUTTON)
        .onTrue(new InstantCommand(() -> {
          arm.setArmTarget(CLIMBER_DOWN_ANGLE_RAD);
        }));


  //have the trigger and button bindings here call the Intake, Shoot, and Eject commands

  //NEW BINDINGS(easier for manipulator)
  //Xbox right trigger axis -> Shoot
  //Xbox left trigger axis -> Intake
  //Xbox right bumper button -> Speaker (arm position)
  //Xbox left bumper button -> Amp (arm position and RPM and Eject)
  //Xbox X button -> Intake(arm position)

    //IMPLEMENTING BINDINGS WHEN MERGED WITH ARM
    /*/Eject also for AMP/*/
    new JoystickButton(manipulatorController, AMP_BUTTON).onTrue(new Eject(intakeShooter));

    /*/Shooting/*/
    new JoystickButton(manipulatorController, SHOOTER_BUTTON).onTrue(new PassToOutake(intakeShooter));

    /*/Intake/*/ 
    new JoystickButton(manipulatorController, INTAKE_BUTTON).onTrue(new Intake(intakeShooter)); //I don't know the UI so this is placeholder
    new JoystickButton(manipulatorController, Button.kLeftStick.value).onTrue(new InstantCommand(() -> {manipulatorController.setRumble(RumbleType.kBothRumble, 1);}));
   // new JoystickButton(manipulatorController, Button.kLeftStick.value).onFalse(new InstantCommand(() -> {manipulatorController.setRumble(RumbleType.kBothRumble, 0);}));
    intakeShooter.setDefaultCommand(new RumbleNote(intakeShooter, manipulatorController));
    //TODO: ask charles if passing in controller is okay
  }

  public Command getAutonomousCommand() {
    //Auto-generated method stub
	//return new PrintCommand("No auto, intakeshooter branch");
  //throw new UnsupportedOperationException("Unimplemented method 'getAutonomousCommand'");

    return Commands.print("No autonomous command configured");
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

  /**
   * Returns zero if a axis input is inside the deadzone
   * 
   * @param hid  The controller/plane joystick the axis is on
   * @param axis The processed axis
   * @return The processed value.
   */
  private double DeadzonedAxis(double axOut) {
    return (-OI.JOY_THRESH < axOut && axOut < OI.JOY_THRESH) ? 0.0 : axOut;
  }

  
}
