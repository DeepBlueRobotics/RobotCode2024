// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

import org.carlmontrobotics.commands.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands.registerCommand;
//import com.pathplanner.lib.auto.NamedCommands;

import java.util.HashMap;

public class RobotContainer {
	//set up subsystems / controllers / limelight


	//auto & paths {

	////AUTO-USABLE COMMANDS
	private final Command[] autoUseableCommands = new Command[] {
		AutoIntakeOnce
		//other commands here
	};
	for (Command cmd : autoUseableCommands) {
		registerCommand(cmd.getName(), (FunctionalCommand) cmd);
	};

	////CREATING PATHS
	private final String[] autoNames = new String[] {/*These are assumed to be equal to the file names*/
		"Penis"
	};
	private final PathPlannerPath[] autoPaths = autoNames.stream().map((name)->fromPathFile(name)).collect(Collectors.toList());

	////CREATE COMMANDS FROM PATHS
	private final AutoBuilder atBuilder = new AutoBuilder();
	private Command[] autoCommands = new Command[] {
		atBuilder.followPath(PathPlannerPath path)
	};

	//}end


  public RobotContainer() {
		//defaultCommands: elevator, dt
		//(pass in controller!)

    configureBindingsDriver();
		configureBindingsManipulator();
  }

  private void configureBindingsDriver() {
		// 4 cardinal directions on arrowpad
		// slowmode toggle on trigger
		// 3 cardinal directions on letterpad
	}

	private void configureBindingsManipulator() {
		// 3 setpositions of elevator on arrowpad
		// intake/outtake on triggers

    //3 setpositions of arm on letterpad
    //right joystick used for manual arm control
	}

  public Command getAutonomousCommand() {
    public Command autoCommand = null;

	//get the funny ports on the robot
	public final DigitalInput[] autoSelectors = new DigitalInput[Math.min(autoNames.length, 26)];
	for(int a = 0; a < autoSelectors.length; a++) autoSelectors[i] = new DigitalInput(i);

	//check which ones are short-circuiting
    for(int i = 0; i < autoSelectors.length; i++) {
      if(!autoSelectors[i].get()) {
        System.out.println("Using Path: " + i);
        autoCommand = autoCommands[i];
        break;
      }
    }

    //return autoPath == null ? new PrintCommand("No Autonomous Routine selected") : autoCommand;
     return autoCommand == null ? new PrintCommand("Auto selector broke :(") : autoCommand;
  }
}
