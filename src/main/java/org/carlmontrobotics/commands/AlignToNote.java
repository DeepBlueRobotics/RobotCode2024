// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Drivetrainc.*;
import static org.carlmontrobotics.Constants.Limelightc.*;

import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.LimelightHelpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToNote extends Command {

    public final TeleopDrive teleopDrive;
    public final Drivetrain drivetrain;

    public final PIDController rotationPID = new PIDController(thetaPIDController[0], thetaPIDController[1],
            thetaPIDController[2]);

    public AlignToNote(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.teleopDrive = (TeleopDrive) drivetrain.getDefaultCommand();

        rotationPID.enableContinuousInput(-180, 180);
        Rotation2d targetAngle = Rotation2d.fromDegrees(drivetrain.getHeading())
                .minus(Rotation2d.fromDegrees(LimelightHelpers.getTX(INTAKE_LL_NAME)));
        rotationPID.setSetpoint(MathUtil.inputModulus(targetAngle.getDegrees(), -180, 180));
        rotationPID.setTolerance(positionTolerance[2], velocityTolerance[2]);
        SendableRegistry.addChild(this, rotationPID);

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        Rotation2d targetAngle = Rotation2d.fromDegrees(drivetrain.getHeading())
                .minus(Rotation2d.fromDegrees(LimelightHelpers.getTX(INTAKE_LL_NAME)));
        rotationPID.setSetpoint(MathUtil.inputModulus(targetAngle.getDegrees(), -180, 180));
        if (teleopDrive == null)
            drivetrain.drive(0, 0, rotationPID.calculate(drivetrain.getHeading()));
        else {
            double[] driverRequestedSpeeds = teleopDrive.getRequestedSpeeds();
            drivetrain.drive(driverRequestedSpeeds[0], driverRequestedSpeeds[1],
                    rotationPID.calculate(drivetrain.getHeading()));
        }
    }

    @Override
    public boolean isFinished() {
        return false;
        // SmartDashboard.putBoolean("At Setpoint", rotationPID.atSetpoint());
        // SmartDashboard.putNumber("Error", rotationPID.getPositionError());
        // return rotationPID.atSetpoint();
    }
}
