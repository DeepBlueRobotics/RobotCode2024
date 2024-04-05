package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.Command;

public class MoveToPos extends Command {
  // return true when close enough
  private final Arm arm;
  private double goal;
  private int index;

  // TODO: don't have an index parameter
  public MoveToPos(Arm armSubsystem, double goal, int index) {
    addRequirements(this.arm = armSubsystem);
    this.goal = goal;
    this.index = index;
  }

  @Override
  public void initialize() {
    Arm.setSelector(index);
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
