// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Drivetrainc.*;

import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.Limelight;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToApriltag extends Command {

        public final TeleopDrive teleopDrive;
        public final Drivetrain drivetrain;
        private Limelight limelight;

        public final PIDController rotationPID = new PIDController(
                        SmartDashboard.getNumber("apriltag align kp",
                                        thetaPIDController[0]),
                        SmartDashboard.getNumber("apriltag align ki",
                                        thetaPIDController[1]),
                        SmartDashboard.getNumber("apriltag align kd",
                                        thetaPIDController[2]));

        double RotationSpeed = 0.0;

        public AlignToApriltag(Drivetrain drivetrain, Limelight limelight, double RotationSpeed) {
                this.limelight = limelight;
                this.drivetrain = drivetrain;
                this.RotationSpeed = RotationSpeed;
                this.teleopDrive = (TeleopDrive) drivetrain.getDefaultCommand();

                rotationPID.enableContinuousInput(-180, 180);
                Rotation2d targetAngle = Rotation2d
                                .fromDegrees(drivetrain.getHeading())
                                .plus(Rotation2d.fromRadians(limelight
                                                .getRotateAngleRadMT2()));
                rotationPID.setSetpoint(MathUtil.inputModulus(
                                targetAngle.getDegrees(), -180, 180));
                rotationPID.setTolerance(
                                SmartDashboard.getNumber(
                                                "apriltag align pos tolerance",
                                                positionTolerance[2]),
                                SmartDashboard.getNumber(
                                                "apriltag align vel tolerance",
                                                velocityTolerance[2]));
                SendableRegistry.addChild(this, rotationPID);
                addRequirements(drivetrain);
        }

        @Override
        public void execute() {
                double kp = SmartDashboard.getNumber("apriltag align kp",
                                rotationPID.getP());
                double ki = SmartDashboard.getNumber("apriltag align ki",
                                rotationPID.getI());
                double kd = SmartDashboard.getNumber("apriltag align kd",
                                rotationPID.getD());

                if (kp != rotationPID.getP())
                        rotationPID.setP(kp);
                if (ki != rotationPID.getI())
                        rotationPID.setI(ki);
                if (kd != rotationPID.getD())
                        rotationPID.setD(kd);

                double posTolerance = SmartDashboard.getNumber(
                                "apriltag align pos tolerance",
                                rotationPID.getPositionTolerance());
                double velTolerance = SmartDashboard.getNumber(
                                "apriltag align vel tolerance",
                                rotationPID.getVelocityTolerance());

                if (posTolerance != rotationPID.getPositionTolerance()
                                || velTolerance != rotationPID
                                                .getVelocityTolerance())
                        rotationPID.setTolerance(posTolerance, velTolerance);

                SmartDashboard.putNumber("apriltag align pos error (rad)",
                                rotationPID.getPositionError());
                SmartDashboard.putNumber("apriltag align vel error (rad/s)",
                                rotationPID.getVelocityError());

                Rotation2d targetAngle = Rotation2d
                                .fromDegrees(drivetrain.getHeading())
                                .plus(Rotation2d.fromRadians(limelight
                                                .getRotateAngleRadMT2()));
                rotationPID.setSetpoint(MathUtil.inputModulus(
                                targetAngle.getDegrees(), -180, 180));
                double rotationDrive =
                                rotationPID.calculate(drivetrain.getHeading());
                if (!limelight.seesTag()) {
                        rotationDrive = RotationSpeed;
                }
                if (rotationPID.atSetpoint())
                        rotationDrive = 0;

                if (teleopDrive == null)
                        drivetrain.drive(0, 0, rotationDrive);
                else {
                        double[] driverRequestedSpeeds =
                                        teleopDrive.getRequestedSpeeds();
                        drivetrain.drive(driverRequestedSpeeds[0],
                                        driverRequestedSpeeds[1],
                                        rotationDrive);
                }
        }

        @Override
        public boolean isFinished() {
                // return false;
                // SmartDashboard.putBoolean("At Setpoint", rotationPID.atSetpoint());
                // SmartDashboard.putNumber("Error", rotationPID.getPositionError());
                return rotationPID.atSetpoint();
        }
}
