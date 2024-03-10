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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//control bindings
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {
  // 1. using GenericHID allows us to use different kinds of controllers
  // 2. Use absolute paths from constants to reduce confusion
  public final GenericHID driverController = new GenericHID(OI.Driver.port);
  public final GenericHID manipulatorController = new GenericHID(OI.Manipulator.port);

  private final IntakeShooter intakeShooter = new IntakeShooter();
  public Arm arm = new Arm();

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
    intakeShooter.setDefaultCommand(new TeleopEffector(
      intakeShooter,
      () -> ProcessedAxisValue(manipulatorController, Axis.kLeftY)
    ));
    arm.setDefaultCommand(new TeleopArm(
      arm,
      () -> ProcessedAxisValue(manipulatorController, Axis.kRightY)
    ));
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
      .onTrue(new ParallelCommandGroup(
        new MoveToPos(arm, Armc.INTAKE_ANGLE_RAD),
        new Intake(intakeShooter)
      ));
    /*/Shooting/*/
    new JoystickButton(manipulatorController, OI.Manipulator.SPEAKER_CLOSE)//aka podium
      .onTrue(new SequentialCommandGroup(
        new MoveToPos(arm, Armc.PODIUM_ANGLE_RAD),
        new RampToRPM(intakeShooter, Effectorc.OUTAKE_RPM_CLOSE),
        new PassToOutake(intakeShooter),
        new WaitCommand(1),
        new InstantCommand(intakeShooter::stopOutake)
      ));
    new JoystickButton(manipulatorController, OI.Manipulator.SPEAKER_SAFE)
      .onTrue(new SequentialCommandGroup(
        new MoveToPos(arm, Armc.SAFE_ZONE_ANGLE_RAD),
        new RampToRPM(intakeShooter, Effectorc.OUTAKE_RPM_SAFE),
        new PassToOutake(intakeShooter),
        new WaitCommand(1),
        new InstantCommand(intakeShooter::stopOutake)
      ));
    new JoystickButton(manipulatorController, OI.Manipulator.AMP)//MELEE ATTACK
      .onTrue(new SequentialCommandGroup(
        new MoveToPos(arm, Armc.AMP_ANGLE_RAD),
        new Eject(intakeShooter)
      ));
    /*/Singulars/*/
    new JoystickButton(manipulatorController, OI.Manipulator.INTAKE_POS)
      .onTrue(new MoveToPos(arm, Armc.INTAKE_ANGLE_RAD));
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
    return DeadzonedAxis(inputProcessing(getStickValue(hid, axis)));
  }

  /**
   * Returns zero if a axis input is inside the deadzone
   *
   * @param hid  The controller/plane joystick the axis is on
   * @param axis The processed axis
   * @return The processed value.
   */
  private double DeadzonedAxis(double axOut) {
    return (Math.abs(axOut) <= OI.JOY_THRESH) ? 0.0 : axOut;
  }


}
