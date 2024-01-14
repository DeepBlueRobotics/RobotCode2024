// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.subsystems;

import com.revrobotics.CANSparkMax;


import org.carlmontrobotics.lib199.MotorControllerFactory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand; 


public class Shooter extends SubsystemBase {
    CANSparkMax leftFlywheelMotor = MotorControllerFactory.createSparkMax(0, MotorConfig.NEO_550);
    CANSparkMax rightFlywheelMotor = MotorControllerFactory.createSparkMax(1,MotorConfig.NEO_550);
    public Shooter() {
    }
    public void intake() {
      //turn wheel backwards to suckkkkkk
    }
    public void isHoldingNote(){

    }
    // method to check if holding note (beambreaker / digital input)

    public void outtake(){
      //sets flywheel to ideal speed
    }
  
    //1 outske
    @Override
    public void periodic() {

     
		}
}
