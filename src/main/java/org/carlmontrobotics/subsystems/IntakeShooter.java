// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.subsystems;

import com.revrobotics.CANSparkMax;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.Constants.IntakeShooter.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand; 


public class IntakeShooter extends SubsystemBase {
    CANSparkMax leftFiringMotor = MotorControllerFactory.createSparkMax(leftFiringPort, MotorConfig.NEO);
    CANSparkMax rightFiringMotor = MotorControllerFactory.createSparkMax(rightFiringPort,MotorConfig.NEO);
    
    CANSparkMax leftPassMotor = MotorControllerFactory.createSparkMax(leftPassPort, MotorConfig.NEO_550);
	CANSparkMax rightPassMotor = MotorControllerFactory.createSparkMax(rightPassPort,MotorConfig.NEO_550);
	
	CANSparkMax leftIntakeMotor = MotorControllerFactory.createSparkMax(leftIntakePort, MotorConfig.NEO_550);
  	CANSparkMax rightIntakeMotor = MotorControllerFactory.createSparkMax(leftIntakePort, MotorConfig.NEO_550);
	
		public IntakeShooter() {}
	
		public void intake() {}//run both intake and passing motors
    
    	public void shoot() {}//run both passing and shooting motors. May need PID for this
		
		public void eject() {}//throw ring onto ground (run all motors in reverse)
	
	
    public void isHoldingNote(){
			// method to check if holding note (beambreaker / digital input)
    }

  
    
    @Override
    public void periodic() {
			//is pid needed? for rpm speeds?
		}
}
