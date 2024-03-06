// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

import org.carlmontrobotics.Constants.OI;
//199 files
import org.carlmontrobotics.commands.*;
import static org.carlmontrobotics.Constants.OI.Manipulator.*;

//subsystems
//import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.IntakeShooter;

//controllers
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
//commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;

//control bindings
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  
  // private final Drivetrain drivetrain = new Drivetrain();

  // 1. using GenericHID allows us to use different kinds of controllers
  // 2. Use absolute paths from constants to reduce confusion
  //private final GenericHID driverController = new GenericHID(OI.Driver.port);
  private final GenericHID manipulatorController = new GenericHID(OI.Manipulator.port);
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
  }
  private void setBindingsDriver() {}
  
  private void setBindingsManipulator() {
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
    new JoystickButton(manipulatorController, Button.kLeftStick.value).onFalse(new InstantCommand(() -> {manipulatorController.setRumble(RumbleType.kBothRumble, 0);}));
    intakeShooter.setDefaultCommand(new RumbleNote(intakeShooter, manipulatorController));
    //TODO: ask charles if passing in controller is okay
  }

  public Command getAutonomousCommand() {
    //Auto-generated method stub
	return new PrintCommand("No auto, intakeshooter branch");
    //throw new UnsupportedOperationException("Unimplemented method 'getAutonomousCommand'");
  }

}
