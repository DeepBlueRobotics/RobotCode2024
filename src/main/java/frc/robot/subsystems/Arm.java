// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Arm extends SubsystemBase {
    public Arm() {
			//arm 
      /*
       have 3 set positions
       Speaker position 
       Amp position
        ground position
       These set positions would also be conrtolled by buttons

       There will also be a manual control for the arm using the right joystick
      */
    }
    public void setArmPos() {
      //Sets arm to the optimal angle for amp, speaker and Ground intake | used to score in amp
      //these values are in constants
      //pass in where scorign and use switch statement
    

    }
    
    ///change to 1 method
    public void moveArm(double manipulatirJoyStick) {
      // move the arm around based off the right joystick movement on the manipulator joystick
      //use the trapezoid thingy from robot code 2023
    }
    
    @Override
    public void periodic() {
     // run moveArm passing in controller here
		}
}
