// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import static org.carlmontrobotics.Constants.IntakeShooter.*;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.Constants;
import org.carlmontrobotics.Constants.IntakeShooter.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

import edu.wpi.first.math.controller.PIDController;
import static org.carlmontrobotics.Constants.IntakeShooter.*;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;
//these 4 imports above are for limelight



public class IntakeShooter extends SubsystemBase {
    CANSparkMax leftShootingMotor = MotorControllerFactory.createSparkMax(leftShootingPort, MotorConfig.NEO);
    CANSparkMax rightShootingMotor = MotorControllerFactory.createSparkMax(rightShootingPort, MotorConfig.NEO);
    CANSparkMax intakeMotor = MotorControllerFactory.createSparkMax(intakePort, MotorConfig.NEO_550);

	private final RelativeEncoder leftShootingEncoder = leftShootingMotor.getEncoder();
	private final RelativeEncoder rightShootingEncoder = rightShootingMotor.getEncoder();

	private final RelativeEncoder intakeMotorEncoder = intakeMotor.getEncoder();
	//todo: figure out way to make it so its not "constants."
	private final SparkPIDController leftShootingMotorPID = leftShootingMotor.getPIDController();
	private final SparkPIDController rightShootingMotorPID = rightShootingMotor.getPIDController();
	private final TimeOfFlight distSensor = new TimeOfFlight(distSensorPort);

	//private final DigitalOutput beamBreak = new DigitalOutput(beamBreakPort);
	//We may use this

		//NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
		//NetworkTableEntry tx = table.getEntry("tx");
		//NetworkTableEntry ty = table.getEntry("ty");
		//NetworkTableEntry ta = table.getEntry("ta");

		//SmartDashboard.putNumber("LimelightX", x);
		//SmartDashboard.putNumber("LimelightY", y);
		//SmartDashboard.putNumber("LimelightArea", area);

		//the above commented out code is for limelight
								
	public IntakeShooter() {}

	public void setRPMIntake(){
		//Intake
	}
    	
    public void setRPMSpeaker(){
		//method that gets the speaker motors up to speed
	}

	public void setRPMAmp(){
		//method that gets the Shooter motors up to a speed to eject a note
	}

	public void setRPMEject(){
		//method that runs the shooting and intake motors backwards
	}
	public boolean isHoldingNote(){
		//return beamBreak.get(); use if using beambraker
		
		//use distance sensor to check if holding note
		//we will use get range();

		/*/we will use an if statement and see if the 
		distance is between two certain numbers.  If the
		distance is between the two certain numbers, this will return true/*/

		//Placeholder
		return true;
	}
	public double getDistanceIn(){
		return 0;
		//TODO: explain in comments
	}

    

    @Override
    public void periodic() {
			//pid needed for rpm speeds

			//double x = tx.getDouble(0.0);
			//double y = ty.getDouble(0.0);
			//double area = ta.getDouble(0.0);
			//the above code is for limelight; idk if i put it in the right spot tho


			//sets goal rpms for different shooting modes for PID
			//setReference​(double AMPLIFIERRPM, CANSparkBase.CANSparkMax.kVelocity);
			//setReference​(double SPEAKERRPM, CANSparkBase.CANSparkMax.kVelocity);

		}
}
