// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Drivetrainc.*;

import java.util.List;

import org.carlmontrobotics.subsystems.Drivetrain;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveToSpeaker extends Command {
  private final Drivetrain dt;
  /** Creates a new MoveToSpeaker. */
  public MoveToSpeaker(Drivetrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.dt = dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(dt.getPose(), new Pose2d(1, 1, Rotation2d.fromDegrees(1)));
    PathPlannerPath path = new PathPlannerPath(bezierPoints, new PathConstraints(autoMaxSpeedMps, autoMaxAccelMps2, maxRCW, 1), new GoalEndState(1, Rotation2d.fromDegrees(1)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
