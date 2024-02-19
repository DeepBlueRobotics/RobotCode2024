// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import static org.carlmontrobotics.Constants.IntakeShooter.*;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.Constants;
import org.carlmontrobotics.Constants.IntakeShooter.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;


public class IntakeShooter extends SubsystemBase {
    //CANSparkMax leftShootingMotor = MotorControllerFactory.createSparkMax(leftShootingPort, MotorConfig.NEO);
    private final CANSparkMax outtakeMotor = MotorControllerFactory.createSparkMax(OuttakePort, MotorConfig.NEO_550);
    private final CANSparkMax intakeMotor = MotorControllerFactory.createSparkMax(IntakePort, MotorConfig.NEO_550);

	//private final RelativeEncoder leftShootingEncoder = leftShootingMotor.getEncoder();
	private final RelativeEncoder outtakeEncoder = outtakeMotor.getEncoder();
	private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS,kV,kA);
	private final RelativeEncoder intakeMotorEncoder = intakeMotor.getEncoder();
	//todo: figure out way to make it so its not "constants."
	//private final SparkPIDController leftShootingMotorPID = leftShootingMotor.getPIDController();
	private final SparkPIDController pid = outtakeMotor.getPIDController();
	private final TimeOfFlight distSensor = new TimeOfFlight(dsPort1);
	TimeOfFlight distSensor2 = new TimeOfFlight(dsPort2);
	private final int[] location = {0,1,2,3};
 	private double DSdepth = 9.97;
  	private double DSdetectdistance =13;

								
	public IntakeShooter() {
		pid.setP(kP);
		pid.setD(kI);
	}
	public void setRMPShooter(int index){


		//pass in location, from the calculate RPM method 
		//   - diagonally, right in front, from wing
		//does the above comment refer to the location of the robot compared to the speaker?
		//set rpm for shooting in speaker depending on location


		//set rpm for 2 different arm shooting positions - armAmpPos and armSpeakerPos
		//when right trigger pressed (speaker), set rpm to speakerRPM
		//when right bumper button pressed (amp), set rpm to ampRPM

		//How shooting will work: arm has buttons/triggers on controller to move it to different positions
		//there will be a separate button/trigger to shoot a note from the end effector regardless of arm position

	}
	public void intake() {
		pid.setReference(intakeRPM, CANSparkBase.ControlType.kVelocity,0,feedforward.calculate(intakeRPM));
	}
	public void setRPMEject(){
		//method that runs the shooting and intake motors backwards
	}
	public double getGamePieceDistanceIn() {
		return Units.metersToInches((distSensor.getRange() - DSdepth) / 1000 /* Convert mm to m */);
	  }
	public double getGamePieceDistanceIn2() {
		return Units.metersToInches((distSensor2.getRange() - DSdepth) / 1000 /* Convert mm to m */);
	}
	public boolean checkNote(){	
		if(getGamePieceDistanceIn() < DSdetectdistance) {
			pid.setReference(-1, CANSparkBase.ControlType.kVelocity,0,feedforward.calculate(-1/60));
			if(getGamePieceDistanceIn2()<DSdetectdistance) {
				pid.setReference(0, CANSparkBase.ControlType.kVelocity,0,feedforward.calculate(0));
				return true;
			}
		
		}
		//use distance sensor to check if holding note
		//we will use get range();

		/*/we will use an if statement and see if the 
		distance is between two certain numbers.  If the
		distance is between the two certain numbers, this will return true/*/
		//Placeholder
		return false;
	}

	public void calculateRPM(/*/location/*/){
		//calculates the speed depending on location and use that in setRPM
		/**
	 	* @param location on the field
		* @return void
	 	*/
		
		
	}
	

    

    @Override
    public void periodic() {
			//pid needed for rpm speeds

			//sets goal rpms for different shooting modes for PID
			//setReference​(double AMPLIFIERRPM, CANSparkBase.CANSparkMax.kVelocity);
			//setReference​(double SPEAKERRPM, CANSparkBase.CANSparkMax.kVelocity);

		}
}
