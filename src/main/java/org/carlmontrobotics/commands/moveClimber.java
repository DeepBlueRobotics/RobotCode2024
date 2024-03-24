// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Armc.GROUND_INTAKE_POS;
import static org.carlmontrobotics.Constants.Armc.HANG_ANGLE_RAD;
import static org.carlmontrobotics.Constants.Armc.LOWER_ANGLE_LIMIT_RAD;
import static org.carlmontrobotics.Constants.Armc.MAX_FF_ACCEL_RAD_P_S;
import static org.carlmontrobotics.Constants.Armc.TRAP_CONSTRAINTS;
import static org.carlmontrobotics.Constants.Armc.UPPER_ANGLE_LIMIT_RAD;

import org.carlmontrobotics.subsystems.Arm;

import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class moveClimber extends Command {
  /** Creates a new moveClimber. */
  private final Arm arm;
  private double goal;

  public moveClimber(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.arm = arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.callDrive = false;
    if (!arm.armClimbing) {
      goal = HANG_ANGLE_RAD;
    } else {
      goal = GROUND_INTAKE_POS;
    }
    arm.armClimbing = !arm.armClimbing;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.drivearm(Math.signum(goal - arm.getArmPos()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setArmTarget(goal);
    arm.callDrive = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return Math.abs(arm.getArmPos() - goal) < .1;
  }
}
