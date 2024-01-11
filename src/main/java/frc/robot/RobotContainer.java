// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
		// 3 setpositions of elevator on letterpad
		// intake/outtake on triggers
	}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
