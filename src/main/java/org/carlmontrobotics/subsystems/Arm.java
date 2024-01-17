// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.subsystems;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.Constants.Arm.*;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class Arm extends SubsystemBase {
    private final CANSparkMax armMotor = MotorControllerFactory.createSparkMax(Constants.Arm.MOTOR_PORT,MotorConfig.NEO);
    private final SimpleMotorFeedforward armFeed = new SimpleMotorFeedforward(Constants.Arm.kS, Constants.Arm.kV);
    private final XboxController controller;
    public Arm(XboxController controller) {
			//arm 
      /*
       have 3 set positions
       Speaker position 
       Amp position
        ground position
       These set positions would also be conrtolled by buttons

       There will also be a manual control for the arm using the right joystick
      */
      this.controller = controller;
    }
	
    public void setArmGoal(double targetPosition, double targetVelocity) {
      //Sets arm to the optimal angle for amp, speaker and Ground intake | used to score in amp
      //these values are in constants
      //pass in where scorign and use switch statement
      targetPosition = getArmClampedGoal(targetPosition);
      
    }
    public static boolean positionForbidden(double armPos) {

      return false;
    }
    public void controllerSetSpeed(double rightJoystick) {
      //move the arm around based off the right joystick movement on the manipulator joystick
      //use the trapezoid thingy from robot code 2023
      
    }
    public double getArmClampedGoal(double goal) {
      return MathUtil.clamp(MathUtil.inputModulus(goal, Constants.Arm.ARM_DICONT_RAD, Constants.Arm.ARM_DICONT_RAD + 2 * Math.PI), Constants.Arm.LOWER_ANGLE, Constants.Arm.UPPER_ANGLE);
    }
    @Override
    public void periodic() {
     // run moveArm passing in controller here
     controllerSetSpeed(controller.getRightY());
		}
}
