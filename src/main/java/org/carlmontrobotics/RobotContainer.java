// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

//199 files
// import org.carlmontrobotics.subsystems.*;
// import org.carlmontrobotics.commands.*;
import static org.carlmontrobotics.Constants.OI;

import org.carlmontrobotics.Constants.OI;
//subsystems
import org.carlmontrobotics.subsystems.Arm;
//import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.IntakeShooter;



//controllers
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.StadiaController.Button;
import edu.wpi.first.wpilibj.XboxController.Axis;

//commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.carlmontrobotics.commands.IntakeRPM;
import org.carlmontrobotics.commands.ShootAmpRPM;
import org.carlmontrobotics.commands.ShootSpeakerRPM;
import org.carlmontrobotics.commands.EjectRPM;




//control bindings
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final Arm arm = new Arm(); 
  IntakeShooter intakeShooter = new IntakeShooter();
  //private final Drivetrain drivetrain = new Drivetrain();  

  //1. using GenericHID allows us to use different kinds of controllers
  //2. Use absolute paths from constants to reduce confusion
  public final GenericHID driverController = new GenericHID(OI.Driver.port);
  public final GenericHID manipulatorController = new GenericHID(OI.Manipulator.port);

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
  

  }
  private void setBindingsDriver() {}
  
  private void setBindingsManipulator() {
  //have the trigger and button bindings here call the Intake, Shoot, and Eject commands 
  
  //old bindings 
  //Xbox right trigger axis -> Speaker (arm position)
  //Xbox left trigger axis -> Intake
  //Xbox right bumper button -> Amp (arm position)
  //Xbox A button -> Eject 
  //Xbox X button -> Shoot

  //NEW BINDINGS(easier for manipulator)
  //Xbox right trigger axis -> Shoot
  //Xbox left trigger axis -> Intake
  //Xbox right bumper button -> Speaker (arm position)
  //Xbox left bumper button -> Amp (arm position)
  //Xbox X button -> Intake(arm position)
  //Xbox A button -> Eject
    
    /*/TODO: look over new JoystickButton(manipulatorController, Button.kLeftBumper.value).onTrue(new IntakeRPM(intakeShooter));
    new JoystickButton(manipulatorController, Button.kLeftBumper.value).onTrue(new ShootAmpRPM(intakeShooter));
    new JoystickButton(manipulatorController, Button.kLeftBumper.value).onTrue(new ShootSpeakerRPM(intakeShooter));/*/ 



    //Intake(placeholder)
    new JoystickButton(manipulatorController, Button.kLeftBumper.value).onTrue(new InstantCommand(() -> intakeShooter.setRPMintake()));
    new JoystickButton(manipulatorController, Button.kLeftBumper.value).onFalse(new InstantCommand(() -> intakeShooter.stopIntake()));
    //Speaker and amp(placeholder)
    new JoystickButton(manipulatorController, Button.kRightBumper.value ).onTrue(new InstantCommand(() -> intakeShooter.setRPMOutake((intakeShooter.calculateDistanceForRPM()))));
    new JoystickButton(manipulatorController, Button.kRightBumper.value).onFalse(new InstantCommand(() -> intakeShooter.stopOutake()));    
    //Eject
    new JoystickButton(manipulatorController, Button.kA.value).onTrue(new InstantCommand(() -> intakeShooter.setRPMEjectOutake()));
    new JoystickButton(manipulatorController, Button.kA.value).onTrue(new InstantCommand(() -> intakeShooter.setRPMEjectIntake()));
 
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
