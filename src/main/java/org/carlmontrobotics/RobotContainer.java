// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;


//199 files
import org.carlmontrobotics.commands.*;
import static org.carlmontrobotics.Constants.*;

import org.carlmontrobotics.Constants.OI;
import org.carlmontrobotics.Constants.OI.*;

//subsystems
import org.carlmontrobotics.subsystems.*;

//controllers
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
//commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//control bindings
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  // 1. using GenericHID allows us to use different kinds of controllers
  // 2. Use absolute paths from constants to reduce confusion
  //private final GenericHID driverController = new GenericHID(OI.Driver.port);
  private final GenericHID manipulatorController = new GenericHID(OI.Manipulator.port);

  private final IntakeShooter intakeShooter = new IntakeShooter();

  public RobotContainer() {
    // setupAutos();
		setDefaultCommands();
		setBindingsDriver();
		setBindingsManipulator();
  }
  
	private void setDefaultCommands() {
    // drivetrain.setDefaultCommand(new TeleopDrive(
    //   drivetrain,
    //   () -> ProcessedAxisValue(driverController, Axis.kLeftY),
    //   () -> ProcessedAxisValue(driverController, Axis.kLeftX),
    //   () -> ProcessedAxisValue(driverController, Axis.kRightX),
    //   () -> driverController.getRawButton(OI.Driver.slowDriveButton)
    // ));
    // intakeShooter.setDefaultCommand(new TeleopEffector(
    //   intakeShooter, 
    //   () -> ProcessedAxisValue(manipulatorController, Axis.kLeftY),
    // ));
    // arm.setDefaultCommand(new TeleopArm(
    //   arm, 
    //   () -> ProcessedAxisValue(manipulatorController, Axis.kRightY)
    // ));
  }
  private void setBindingsDriver() {}
  
  private void setBindingsManipulator() {
    //NEW BINDINGS(easier for manipulator)
    //Xbox left joy Y axis -> raw Intake/Outtake control
    //Xbox right joy Y axis -> raw Arm control
    //Xbox right trigger axis -> Intake pos + intake
    //Xbox left trigger axis -> amp pos , eject into amp
    //Xbox left bumper button -> CLOSE Speaker pos , Fire
    //Xbox right bumper button -> SAFE  Speaker pos , Fire
    //Xbox X button -> goto Intake pos
    //Xbox Y button -> Eject rpm
  
    /*/Multi-commands/*/
    new JoystickButton(manipulatorController, OI.Manipulator.INTAKE)
      .onTrue(new SequentialCommandGroup(
        //needs movearm
        new Intake(intakeShooter)
      ));
    /*/Shooting/*/
    new JoystickButton(manipulatorController, OI.Manipulator.SPEAKER_CLOSE)
      .onTrue(new SequentialCommandGroup(
        //needs movearm
        //needs ramprpm
        new PassToOutake(intakeShooter)
      ));
    new JoystickButton(manipulatorController, OI.Manipulator.SPEAKER_SAFE)
      .onTrue(new SequentialCommandGroup(
        //needs movearm
        //needs ramprpm
        new PassToOutake(intakeShooter)
      ));
    new JoystickButton(manipulatorController, OI.Manipulator.AMP)
      .onTrue(new SequentialCommandGroup(
        //needs movearm
        //needs ramprpm
        new Eject(intakeShooter)
      ));
    /*/Singulars/*/ 
    /*new JoystickButton(manipulatorController, OI.Manipulator.INTAKE_POS)
      .onTrue(NEEDS MOVEARM);*/
    new JoystickButton(manipulatorController, OI.Manipulator.EJECT_RPM)
      .onTrue(new Eject(intakeShooter));
    // new JoystickButton(manipulatorController, Button.kLeftStick.value)
    //   .onTrue(new InstantCommand(() -> {manipulatorController.setRumble(RumbleType.kBothRumble, 1);}));
   
    
    //TODO: ask charles if passing in controller is okay
  }

  public Command getAutonomousCommand() {
    //Auto-generated method stub
	return new PrintCommand("No auto, intakeshooter branch");
    //throw new UnsupportedOperationException("Unimplemented method 'getAutonomousCommand'");
  }

}
