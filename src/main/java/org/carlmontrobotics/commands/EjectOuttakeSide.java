package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Armc.SMART_CURRENT_LIMIT_TIMEOUT;

import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class EjectOuttakeSide extends Command {
    private IntakeShooter intakeShooter;
    private Timer timer = new Timer();
    public EjectOuttakeSide(IntakeShooter intakeShooter) {
        this.intakeShooter = intakeShooter;
        addRequirements(intakeShooter);
    }
    @Override
    public void initialize() {
        intakeShooter.setMaxIntake(1);
        intakeShooter.setMaxOuttake(0.6);
        timer.reset();
        timer.start();
    }
    @Override
    public void execute(){ 


    }
    @Override
    public void end(boolean interrupted) {
        intakeShooter.stopIntake();
        intakeShooter.stopOuttake();
        timer.stop();
    }
    @Override
    public boolean isFinished() {
        return false;
        // return timer.get() >= 4 || (!intakeShooter.intakeDetectsNote() &&
        // !intakeShooter.outtakeDetectsNote());
    }
}
