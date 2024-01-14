// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
	//set up subsystems / controllers / limelight
	
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
     return PrintCommand("null :(");
  }
}
