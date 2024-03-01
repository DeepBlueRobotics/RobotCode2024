// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;
import org.carlmontrobotics.subsystems.Arm;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class armAmpPos extends Command {
  /** Creates a new armPosAmp. */
  private Arm arm;
  private Timer armProfileTimer = new Timer();
  public armAmpPos(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armProfileTimer.reset();
    armProfileTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TrapezoidProfile.State setpoint = arm.calculateSetPoint(armProfileTimer.get(), arm.getCurrentArmState(), 1);
    arm.COMBINE_PID_FF_TRAPEZOID(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armProfileTimer.stop();
    armProfileTimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
