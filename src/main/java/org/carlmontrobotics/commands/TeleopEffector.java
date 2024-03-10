// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import org.carlmontrobotics.Constants;
import static org.carlmontrobotics.Constants.Armc.*;
import static org.carlmontrobotics.Constants.Effectorc.MANUAL_RPM_MAX;

import java.util.function.DoubleSupplier;

import org.carlmontrobotics.subsystems.Arm;
import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class TeleopEffector extends Command {
  private final DoubleSupplier joystick;
  private final IntakeShooter intake;

  /** Creates a new ArmTeleop. */
  public TeleopEffector(IntakeShooter effector, DoubleSupplier joystickSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intake = effector);
    joystick = joystickSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setRPMIntake(MANUAL_RPM_MAX * joystick.getAsDouble());
    intake.setRPMOutake(MANUAL_RPM_MAX * joystick.getAsDouble());
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
