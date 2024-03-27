package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class GETOUT extends Command{
    private IntakeShooter intakeShooter;
    Timer timer =  new Timer(); 
    public GETOUT(IntakeShooter intakeShooter) {
        this.intakeShooter = intakeShooter;
    }
    @Override
    public void initialize() {
        timer.start();
        intakeShooter.setMaxIntake(1);
    }
    @Override
    public void execute() {

    }
    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
        intakeShooter.setCurrentLimit(20);
    }
    @Override
    public boolean isFinished() {
        return timer.get() > 1.5;
    }
}
