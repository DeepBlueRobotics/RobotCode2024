// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.Arm;
import org.carlmontrobotics.subsystems.Limelight;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AimArmSpeaker extends Command {
  private final Arm arm;
  private final Limelight ll;

  /** Creates a new AimOuttakeSpeaker. */
  public AimArmSpeaker(Arm arm, Limelight ll) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.arm = arm);
    this.ll = ll;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double goal = ll.getArmAngleToShootSpeakerRad();
    arm.setArmTarget(goal);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
