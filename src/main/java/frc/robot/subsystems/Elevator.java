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

public class Elevator extends SubsystemBase {
    public Elevator() {
    }
    public void setHighPosition() {
      //Sets the elevator to the highest position | used to set the elevator to climb the chain

    }
    public void setDefaultPosition() {
      //Sets ground/low position
    }
    public void setLowPosition() {
      //Sets the elevator to lowest position
    }
    @Override
    public void periodic() {
     
		}
}
