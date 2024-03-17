package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeTesting extends Command {
    private IntakeShooter intakeShooter;
    private int startingRPMWithNote = 5000;
    private double increaseRPM = 500.0;
    private int index = 0;
    public IntakeTesting(IntakeShooter intakeShooter) {
        this.intakeShooter = intakeShooter;
        addRequirements(intakeShooter);
    }
    @Override
    public void initialize() {
        intakeShooter.setRPMIntake(startingRPMWithNote);
    }
    @Override
    public void execute() {
        while(!intakeShooter.outakeDetectsNote()) {
            intakeShooter.setRPMIntake(startingRPMWithNote+increaseRPM*index);
            index++;
            SmartDashboard.putNumber("RPM Used", increaseRPM*index + startingRPMWithNote);

        }
    }
    @Override
    public void end(boolean interrupted) {
        intakeShooter.stopIntake();
        SmartDashboard.putNumber("finalRPMUsed", increaseRPM*index + startingRPMWithNote);
    }
    public boolean isFinished() {
        return intakeShooter.outakeDetectsNote();
    }
}