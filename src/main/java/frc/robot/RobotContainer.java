// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
	//set up subsystems / controllers / limelight
	
	public final DigitalInput[] autoSelectors;
	
	autoSelectors = new DigitalInput[Math.min(/*amount-of-autos*/, 26)];
	for(int i = 0; i < autoSelectors.length; i++) autoSelectors[i] = new DigitalInput(i);
	
	
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
    Command autoPath = null;
    //PPRobotPath autoPath = new PPRobotPath("Basic 7", drivetrain, false, eventMap);
    // Command[] autoPath2 = {
    //   new PPRobotPath("Mid Basic 3", drivetrain, false, eventMap).getPathCommand(true, true), 
    //   new PPRobotPath("Mid Basic 4", drivetrain, false, eventMap).getPathCommand(false, true)
    // };
    // Command[] commands = {
    //   stopDt(),
    //   new WaitCommand(0)
    // };
    // SequentialCommandGroup autoCommand = new SequentialCommandGroup();
    // for (int i = 0; i < autoPath2.length; i++) {
    //   autoCommand.addCommands(autoPath2[i]);
    //   //autoCommand.addCommands(commands[i]);
    // }

    for(int i = 0; i < autoSelectors.length; i++) {
      if(!autoSelectors[i].get()) {
        System.out.println("Using Path: " + i);
        autoPath = autoPaths[i];
        break;
      }
    }

    //return autoPath == null ? new PrintCommand("No Autonomous Routine selected") : autoCommand;
     return autoPath == null ? new PrintCommand("null :(") : autoPath; 
  }
}
