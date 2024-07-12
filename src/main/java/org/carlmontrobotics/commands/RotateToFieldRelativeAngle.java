package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Drivetrainc.*;

import org.carlmontrobotics.subsystems.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateToFieldRelativeAngle extends Command {

    public final TeleopDrive teleopDrive;
    public final Drivetrain drivetrain;

    public final PIDController rotationPID = new PIDController(thetaPIDController[0], thetaPIDController[1],
            thetaPIDController[2]);

    public RotateToFieldRelativeAngle(Rotation2d angle, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.teleopDrive = (TeleopDrive) drivetrain.getDefaultCommand();

        rotationPID.enableContinuousInput(-180, 180);
        rotationPID.setSetpoint(MathUtil.inputModulus(angle.getDegrees(), -180, 180));
        rotationPID.setTolerance(positionTolerance[2], velocityTolerance[2]);
        SendableRegistry.addChild(this, rotationPID);
        // SmartDashboard.pu

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
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
        //SmartDashboard.putBoolean("At Setpoint", rotationPID.atSetpoint());
        //SmartDashboard.putNumber("Error", rotationPID.getPositionError());
        return rotationPID.atSetpoint();
    }
}