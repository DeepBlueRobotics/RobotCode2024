// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;


import java.util.function.DoubleSupplier;

import org.carlmontrobotics.subsystems.Arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ArmTeleop extends CommandBase {
  private final DoubleSupplier joystick; 
  private final Arm armSubsystem;
  /** Creates a new ArmTeleop. */
  public ArmTeleop(DoubleSupplier joystickSupplier, Arm armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    joystick = joystickSupplier;
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double value = joystick.getAsDouble();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
