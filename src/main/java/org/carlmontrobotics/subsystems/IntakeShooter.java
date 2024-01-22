// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//TODO: change motor names to shooter
package org.carlmontrobotics.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import static org.carlmontrobotics.Constants.IntakeShooter.*;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.Constants;
import org.carlmontrobotics.Constants.IntakeShooter.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand; 
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;




public class IntakeShooter extends SubsystemBase {
    CANSparkMax leftFiringMotor = MotorControllerFactory.createSparkMax(leftFiringPort, MotorConfig.NEO);
    CANSparkMax rightFiringMotor = MotorControllerFactory.createSparkMax(rightFiringPort, MotorConfig.NEO);
    CANSparkMax intakeMotor = MotorControllerFactory.createSparkMax(intakePort, MotorConfig.NEO_550);

	private final RelativeEncoder leftFiringEncoder = leftFiringMotor.getEncoder();
	private final RelativeEncoder rightFiringEncoder = rightFiringMotor.getEncoder();
	private final RelativeEncoder intakeMotorEncoder = intakeMotor.getEncoder();
	//todo: figure out way to make it so its not "constants."
	private final SparkPIDController leftFiringMotorPID = leftFiringMotor.getPIDController();
	private final SparkPIDController rightFiringMotorPID = rightFiringMotor.getPIDController();
	private final DigitalOutput beamBreak = new DigitalOutput(beamBreakPort);

	
	
	public IntakeShooter() {}

	public void setRPMIntake(){
		//Intake
	}
    
	//TODO: make setRPMShooter and setRPMEject the same method
    public void setRPMSpeaker(){
		//method that gets the speaker motors up to speed
	}

	public void setRPMAmp(){
		//method that gets the Shooter motors up to a speed to eject a note
	}
	public boolean isHoldingNote(){
		//use beam break to check if holding note
		return beamBreak.get();

	}
	public double getDistanceIn(){
		return 0;
		//TODO: explan in comments
	}

    

    @Override
    public void periodic() {
			//pid needed for rpm speeds
		}
}
