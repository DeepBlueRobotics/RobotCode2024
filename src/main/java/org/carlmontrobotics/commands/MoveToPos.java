package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.Command;

public class MoveToPos extends Command {
  // return true when close enough
  private final Arm arm;
  private double goal;

  public MoveToPos(Arm armSubsystem, double goal) {
    addRequirements(this.arm = armSubsystem);
    this.goal = goal;
  }

  @Override
  public void initialize() {
    arm.setArmTarget(goal);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.armAtSetpoint();
  }
}