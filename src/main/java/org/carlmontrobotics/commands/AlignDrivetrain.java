// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import java.util.Optional;

import org.carlmontrobotics.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class AlignDrivetrain extends ProxyCommand {
  static double blueSpeakerX = 0.14;
  static double blueSpeakerY = 5.54;

  static double redSpeakerX = 16.36;
  static double redSpeakerY = 5.51;

  public AlignDrivetrain(Drivetrain dt) {
    super(() -> {
      Optional<Alliance> allianceSide = DriverStation.getAlliance();
      if (allianceSide.get() == Alliance.Red) {
        // double redAngle = Math.atan2(redSpeakerY-dt.getPose().getY(),
        // dt.getPose().getX());
        return new RotateToFieldRelativeAngle(new Rotation2d(
            dt.getPose().getX(), blueSpeakerY - dt.getPose().getY()), dt);

      } else if (allianceSide.get() == Alliance.Blue) {
        // double blueAngle = Math.atan2(blueSpeakerY-dt.getPose().getY(),
        // dt.getPose().getX());
        return new RotateToFieldRelativeAngle(
            new Rotation2d(redSpeakerX - dt.getPose().getX(),
                redSpeakerY - dt.getPose().getY()),
            dt);

      }
      // create an if statement based on alliance side
      return new InstantCommand();
    });
  }
}
