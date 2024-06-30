package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class PassToOuttake extends Command {
    private IntakeShooter intakeShooter;

    public PassToOuttake(IntakeShooter intakeShooter) {
        this.intakeShooter = intakeShooter;
    }

    @Override
    public void initialize() {

        intakeShooter.motorSetIntake(0.6);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        intakeShooter.resetCurrentLimit();
    }

    @Override
    public boolean isFinished() {
        return intakeShooter.intakeDetectsNote()
                && intakeShooter.outtakeDetectsNote();
    }
}
