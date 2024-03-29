// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Armc.*;

import java.util.function.DoubleSupplier;

import org.carlmontrobotics.subsystems.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopArm extends Command {
  private final DoubleSupplier joystick;
  private final Arm armSubsystem;
  private double lastTime = 0;

  private TrapezoidProfile.State goalState;

  /** Creates a new ArmTeleop. */
  public TeleopArm(Arm armSubsystem, DoubleSupplier joystickSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.armSubsystem = armSubsystem);
    joystick = joystickSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    goalState = new TrapezoidProfile.State(armSubsystem.getArmPos(), armSubsystem.getArmVel());
    lastTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // use trapazoid math and controllerMoveArm method from arm subsytem to apply
    // voltage to the motor
    double speeds = getRequestedSpeeds();

    if (speeds == 0) {// if no input, don't set any goals.
      lastTime = Timer.getFPGATimestamp();// update deltaT even when not running
      return;
    }

    double currTime = Timer.getFPGATimestamp();
    double deltaT = currTime - lastTime;// only move by a tick of distance at once
    lastTime = currTime;

    double goalArmRad = goalState.position + speeds * deltaT;// speed*time = dist

    goalArmRad = MathUtil.clamp(goalArmRad, LOWER_ANGLE_LIMIT_RAD, UPPER_ANGLE_LIMIT_RAD);
    goalArmRad = MathUtil.clamp(goalArmRad,
        armSubsystem.getArmPos() + Math.pow(armSubsystem.getMaxVelRad(), 2) / MAX_FF_ACCEL_RAD_P_S,
        armSubsystem.getArmPos() - Math.pow(armSubsystem.getMaxVelRad(), 2) / MAX_FF_ACCEL_RAD_P_S);

    goalState.position = goalArmRad;
    goalState.velocity = 0;
    // don't put in constants bc it's always zero
    armSubsystem.setArmTarget(goalState.position);
  }

  public double getRequestedSpeeds() {
    return armSubsystem.getMaxAccelRad() * joystick.getAsDouble();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
