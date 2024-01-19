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
      //pass in where scorign and use switch statement to alternate between each angle needed
      targetPosition = getArmClampedGoal(targetPosition);
      
    }
    public void controllerMoveArm(double rightJoystick) {
      /*
      // move the arm around based off the right joystick movement on the manipulator joystick
      //use the trapezoid thingy from robot code 2023
      //Math below | Summary: Take in controller Y axis as a double then calculate amount of volts needed to pass to the arm and when to stop based off of the controller movement. Done so by finding the constraints of the arm, translating the controller numbers, and finding how many volts and when to stop using feedvolts and PID
        double kgv = getKg();
        double armFeedVolts = kgv * getCoM().getAngle().getCos() + armFeed.calculate(state.velocity, 0);
        double armPIDVolts = armPID.calculate(getArmPos(), state.position);
        if ((getArmPos() > ARM_UPPER_LIMIT_RAD && state.velocity > 0) ||
            (getArmPos() < ARM_LOWER_LIMIT_RAD && state.velocity < 0)) {
              forbFlag = true;  
            armFeedVolts = kgv * getCoM().getAngle().getCos() + armFeed.calculate(0, 0);
            */

      
    }
    public double getArmClampedGoal(double goal) {
      //Find the limits of the arm. Used to move it and ensure that the arm does not move past the amount
      return MathUtil.clamp(MathUtil.inputModulus(goal, Constants.Arm.ARM_DICONT_RAD, Constants.Arm.ARM_DICONT_RAD + 2 * Math.PI), Constants.Arm.LOWER_ANGLE, Constants.Arm.UPPER_ANGLE);
    }
    @Override
    public void periodic() {
     // run moveArm passing in controller here
     controllerMoveArm(controller.getRightY());
		}
}
