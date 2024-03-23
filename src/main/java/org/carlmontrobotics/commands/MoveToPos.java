package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Effectorc.*;
import static org.carlmontrobotics.Constants.Led.*;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.Arm;
import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
public class MoveToPos extends Command {
    //return true when close enough
    private final Arm arm;
    private double goal;

    public MoveToPos(Arm armSubsystem, double goal) {
        addRequirements(this.arm = armSubsystem);
        this.goal=goal;
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
