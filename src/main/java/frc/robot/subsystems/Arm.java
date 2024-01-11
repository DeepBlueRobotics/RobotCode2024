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
			//arm AND wrist, if it happens
      /*
       have 2 set positions
       The first would face intake direction and the second would face the opposite
       Both should keep the same angle
       These set positions would also be conrtolled by buttons
      */
    }

    @Override
    public void periodic() {
     
		}
}
