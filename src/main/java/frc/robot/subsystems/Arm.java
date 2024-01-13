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
    CANSparkMax armLeftMotor = MotorControllerFactory.createSparkMax(2,MotorConfig.NEO);
    CANSparkMax armRightMotor = MotorControllerFactory.createSparkMax(3,MotorConfig.NEO);
    //^might change to neo vortex
    public Arm() {
			//arm AND wrist, if it happens
      /*
       have 3 set positions
       Speaker position 
       Amp position
       Trap possition
       These set positions would also be conrtolled by buttons

       There will also be a manual control for the arm using the right joystick
      */
    }
    public void setAmpPos() {
      //Sets arm to the optimal angle for amp | used to score in amp

    }
    public void setSpeakerPos() {
      //Sets arm to the optimal angle for speaker | used to score in speaker
    }
    public void setTrapPos() {
      //Sets arm to the optimal angle for trap | used to score in trap
    }
    public void moveArm(double manipulatirJoyStick) {
      // move the arm around based off the right joystick movement on the manipulator joystick
    }
    @Override
    public void periodic() {
     // run moveArm passing in controller here
		}
}
