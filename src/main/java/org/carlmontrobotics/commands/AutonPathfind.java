// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Drivetrainc.maxSpeed;
import static org.carlmontrobotics.Constants.Drivetrainc.swerveRadius;
import static org.carlmontrobotics.Constants.Drivetrainc.thetaPIDController;
import static org.carlmontrobotics.Constants.Drivetrainc.xPIDController;
import static org.carlmontrobotics.Constants.Drivetrainc.Autoc.*;

import org.carlmontrobotics.Constants.Drivetrainc.Autoc;
import org.carlmontrobotics.Robot;
import org.carlmontrobotics.subsystems.Drivetrain;

import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonPathfind extends Command {

    private final Drivetrain drivetrain;
    private final PathfindThenFollowPathHolonomic pathfinder;

    public AutonPathfind(Drivetrain drivetrain, String pathName) {
        // Use addRequirements() here to declare subsystem dependencies.

        addRequirements(this.drivetrain = drivetrain);
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        this.pathfinder = new PathfindThenFollowPathHolonomic(path,
                pathConstraints, drivetrain::getPose, drivetrain::getSpeeds,
                (ChassisSpeeds chassisSpeeds) -> {
                    drivetrain.drive(drivetrain.getKinematics()
                            .toSwerveModuleStates(chassisSpeeds));
                },
                new HolonomicPathFollowerConfig(
                        new PIDConstants(xPIDController[0], xPIDController[1],
                                xPIDController[2], 0), // translation (drive) pid vals
                        new PIDConstants(thetaPIDController[0],
                                thetaPIDController[1], thetaPIDController[2],
                                0), // rotation pid vals
                        maxSpeed, swerveRadius, Autoc.replanningConfig,
                        Robot.robot.getPeriod()// robot period
                ), () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent())
                        return alliance.get() == DriverStation.Alliance.Red;
                    // else:
                    return false;
                }, drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pathfinder.initialize();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pathfinder.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        pathfinder.end(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return pathfinder.isFinished();
    }
}
