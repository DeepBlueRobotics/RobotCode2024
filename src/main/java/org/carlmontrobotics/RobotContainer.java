// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

import org.carlmontrobotics.Constants.OI;
import org.carlmontrobotics.Constants.OI.Manipulator;
//199 files
import org.carlmontrobotics.commands.*;
import static org.carlmontrobotics.Constants.OI.Manipulator.*;

import java.util.function.BooleanSupplier;

//subsystems
//import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.IntakeShooter;

//controllers
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
//commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
//control bindings
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  
  // private final Drivetrain drivetrain = new Drivetrain();

  // 1. using GenericHID allows us to use different kinds of controllers
  // 2. Use absolute paths from constants to reduce confusion
  // private final GenericHID driverController = new GenericHID(OI.Driver.port);
  private final GenericHID manipulatorController = new GenericHID(OI.Manipulator.port);
  private final IntakeShooter intakeShooter = new IntakeShooter();
  public RobotContainer() {
    // defaultCommands: elevator, dt
    // (pass in controller!)

    // setupAutos();
    setDefaultCommands();
    setBindingsDriver();
    setBindingsManipulator();
  }

  private void setDefaultCommands() {
    // drivetrain.setDefaultCommand(new TeleopDrive(
    // drivetrain,
    // (DoubleSupplier) () -> ProcessedAxisValue(driverController, Axis.kLeftY),
    // (DoubleSupplier) () -> ProcessedAxisValue(driverController, Axis.kLeftX),
    // (DoubleSupplier) () -> ProcessedAxisValue(driverController, Axis.kRightX),
    // (BooleanSupplier)() ->
    // driverController.getRawButton(OI.Driver.slowDriveButton)
    // ));
  }

  private void setBindingsDriver() {
  }

  private void setBindingsManipulator() {
      BooleanSupplier isIntake = () -> new JoystickButton(manipulatorController, Manipulator.INTAKE_BOOLEAN_SUPPLIER).getAsBoolean();
      BooleanSupplier isOuttake = () -> new JoystickButton(manipulatorController, Manipulator.SHOOTER_BOOLEAN_SUPPLIER).getAsBoolean();


    // have the trigger and button bindings here call the Intake, Shoot, and Eject
    // commands
    // NEW BINDINGS(easier for manipulator)
    // Xbox right trigger axis -> Shoot
    // Xbox left trigger axis -> Intake
    // Xbox right bumper button -> Speaker (arm position)
    // Xbox left bumper button -> Amp (arm position and RPM and Eject)
    // Xbox X button -> Intake(arm position)

    /* /Eject also for AMP/ */
    new JoystickButton(manipulatorController, AMP_BUTTON).onTrue(new Eject(intakeShooter));

    /* /Shooting/ */
    //new JoystickButton(manipulatorController, SHOOTER_BUTTON).onTrue(new PassToOutake(intakeShooter));

    /* /Intake/ */
    //new JoystickButton(manipulatorController, INTAKE_BUTTON).onTrue(new Intake(intakeShooter)); // I don't know the UI
                                                                                                // so this is
                                                                                                // placeholder
    new JoystickButton(manipulatorController, Button.kLeftStick.value).onTrue(new InstantCommand(() -> {
      manipulatorController.setRumble(RumbleType.kBothRumble, 1);
    }));

    axisTrigger(manipulatorController, Manipulator.SHOOTER_BUTTON)//.and(isOuttake)
      .onTrue(new ConditionalCommand(
        new PassToOutake(intakeShooter), 
        new InstantCommand(), 
        isOuttake
      ));

    Trigger axisTrigger(manipulatorController, Manipulator.INTAKE_BUTTON)//.and(isIntake)
      .onTrue(new ConditionalCommand(
        new Intake(intakeShooter), 
        new InstantCommand(), 
        isIntake
      ));

    // TODO: ask charles if passing in controller is okay
  }

  public Command getAutonomousCommand() {
    // Auto-generated method stub
    return new PrintCommand("No auto, intakeshooter branch");
    // throw new UnsupportedOperationException("Unimplemented method
    // 'getAutonomousCommand'");
  
  }

  private double getStickValue(GenericHID stick, Axis axis) {

    return stick.getRawAxis(axis.value) * (axis == Axis.kLeftY || axis == Axis.kRightY ? -1 : 1);
  }

  private Trigger axisTrigger(GenericHID stick, Axis axis) {
    return new Trigger(() -> Math.abs(getStickValue(stick, axis)) > OI.MIN_AXIS_TRIGGER_VALUE);
  }

}
