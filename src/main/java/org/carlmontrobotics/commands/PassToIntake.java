package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.IntakeShooter;
import edu.wpi.first.wpilibj2.command.Command;

public class PassToIntake extends Command{
    private IntakeShooter intakeShooter;
    public PassToIntake(IntakeShooter intakeShooter) {
        this.intakeShooter = intakeShooter;
    }
    @Override
    public void execute() {
        intakeShooter.motorSetIntake(-0.6);
        intakeShooter.motorSetOutake(-0.5);
    }
    @Override
    public void end(boolean interrupted) {
        intakeShooter.stopIntake();
        intakeShooter.stopOutake();
    }
    
    @Override
    public boolean isFinished() {
        return intakeShooter.intakeDetectsNote() && !intakeShooter.outakeDetectsNote();
    }
}
