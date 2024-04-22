// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ProxyCommand;

public class AlignFromPodium extends ProxyCommand {
     public AlignFromPodium(Drivetrain dt) {
          super(() -> {
              Rotation2d fieldOrientedTargetAngle;
               if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {fieldOrientedTargetAngle = Rotation2d.fromRadians(Math.atan(1.38/2.64));}
               // TODO: Make Math.atan a constant
               else fieldOrientedTargetAngle = Rotation2d.fromRadians(Math.atan(-1.38/2.64));
               return new RotateToFieldRelativeAngle(fieldOrientedTargetAngle, dt);
          });
     }
}