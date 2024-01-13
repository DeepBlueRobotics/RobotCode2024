// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;


import org.carlmontrobotics.lib199.MotorControllerFactory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class Flywheel extends SubsystemBase {


    public Flywheel() {
    }
    public void intake() {
      //turn wheel backwards to suckkkkkk
    }
    public void outtake() {
      //set front flywheel to max then wait until proper velocity then turn on back flywheel
    }
    public void isHoldingNote(){
      
    }
    // method to check if holding note (beam breaker/digital input)
    
    @Override
    public void periodic() {

     
		}
}
