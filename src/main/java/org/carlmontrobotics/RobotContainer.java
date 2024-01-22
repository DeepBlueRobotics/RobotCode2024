// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

//199 files
// import org.carlmontrobotics.subsystems.*;
import org.carlmontrobotics.commands.*;
import org.carlmontrobotics.subsystems.Drivetrain;

import static org.carlmontrobotics.Constants.OI;

//controllers
import edu.wpi.first.wpilibj.DigitalInput;
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

//pathplanner
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {
	//set up subsystems / controllers / limelight
  Drivetrain dt = new Drivetrain();

  /* These must be equal to the pathPlanner path names from the GUI! */
  // Order matters - but the first one is index 1 on the physical selector - index 0 is reserved for null command.
  private final String[] autoNames = new String[] {
    "Peepee"
  };
  private Command[] autoCommands;

  public RobotContainer() {
		//defaultCommands: elevator, dt
		//(pass in controller!)

    setupAutos();
		setDefaultCommands();
		setBindingsDriver();
		setBindingsManipulator();
  }

  private void setupAutos() {
    ////AUTO-USABLE COMMANDS
    NamedCommands.registerCommand("AutoIntakeOnce", new AutoIntakeOnce());

    /*
    ////CREATING PATHS
    ArrayList<PathPlannerPath> autoPaths = new ArrayList<PathPlannerPath>();
    for (String name : autoNames) {
      autoPaths.add(PathPlannerPath.fromPathFile(name));
    }


    //AutoBuilder is setup in the drivetrain.

    //note: is it .followPath or .buildAuto(name) + PathPlannerAuto​(autoName) ???
    ////CREATE COMMANDS FROM PATHS
    autoCommands = (Command[]) autoPaths.stream().map(
      (PathPlannerPath path)->AutoBuilder.followPath(path)
      ).collect(Collectors.toList()).toArray();
    */
    //}end
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
  private void setBindingsDriver() {
		// 4 cardinal directions on arrowpad
		// slowmode toggle on trigger
		// 3 cardinal directions on letterpad
	}
  private void setBindingsManipulator() {
		// 3 setpositions of elevator on arrowpad
		// intake/outtake on triggers

    //3 setpositions of arm on letterpad
    //right joystick used for manual arm control
	}

  public Command getAutonomousCommand() {
    //get the funny ports on the robot
    DigitalInput[] autoSelectors = new DigitalInput[Math.min(autoNames.length, 10)];
    for(int a = 0; a < autoSelectors.length; a++) autoSelectors[a] = new DigitalInput(a);

    //check which ones are short-circuiting
      for(int i = 1; i < autoSelectors.length; i++) { /* skip index 0, reserved for null auto */
        if(!autoSelectors[i].get()) {
          String name = autoNames[i-1];
          System.out.println("Using Path " + i + ": " + name);
          return new PathPlannerAuto(name);
        }
      }

    //return autoPath == null ? new PrintCommand("No Autonomous Routine selected") : autoCommand;
    return new PrintCommand("No Auto selected | Auto selector broke :(");
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
