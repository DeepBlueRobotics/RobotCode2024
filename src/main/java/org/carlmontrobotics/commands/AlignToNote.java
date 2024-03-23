// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Limelightc.*;
import org.carlmontrobotics.subsystems.Drivetrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import org.carlmontrobotics.subsystems.LimelightHelpers;

public class AlignToNote extends ProxyCommand {

     public AlignToNote(Drivetrain dt) {
          super(() -> {
               Rotation2d fieldOrientedTargetAngle = Rotation2d.fromDegrees(LimelightHelpers.getTX(INTAKE_LL_NAME)).plus(Rotation2d.fromDegrees(dt.getHeading()));
               return new RotateToFieldRelativeAngle(fieldOrientedTargetAngle, dt);
          });
     }
}
