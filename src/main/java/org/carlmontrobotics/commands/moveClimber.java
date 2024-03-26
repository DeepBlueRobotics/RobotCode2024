// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import static edu.wpi.first.units.Units.Volt;
import static org.carlmontrobotics.Constants.Armc.*;

import org.carlmontrobotics.subsystems.Arm;

import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class moveClimber extends Command {
  /** Creates a new moveClimber. */
  private final Arm arm;
  private double goal = HANG_ANGL_RAD;

  public moveClimber(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.arm = arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setBooleanDrive(false);//turn normal arm periodic off
    //arm.setArmTarget(goal);
    SmartDashboard.putNumber("climber volts", 0);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.drivearm(SmartDashboard.getNumber("climber volts", 0));
    // double err = goal - arm.getArmPos();
    // arm.drivearm(//equivalent to a clamped P controller with KS
    //   Math.signum(err) * //direction control
    //       MathUtil.clamp(
    //         Math.abs(err)*2,//.5rad err -> 1 speed
    //         .2,
    //         .1
    //       )
    // );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.driveMotor(Volt.of(0));
    arm.setArmTarget(arm.getArmPos());
    arm.setBooleanDrive(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(arm.getArmPos() - goal) < .1;
  }
}
