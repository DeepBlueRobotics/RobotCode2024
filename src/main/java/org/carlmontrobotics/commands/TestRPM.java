package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TestRPM extends Command {
    private IntakeShooter intakeShooter;
    private double goalRPM;

    public TestRPM(IntakeShooter intakeShooter) {
        this.intakeShooter = intakeShooter;
        addRequirements(intakeShooter);
    }

    @Override
    public void initialize() {
        intakeShooter.setMaxOuttake(1);
    }

    @Override
    public void execute() {
        goalRPM = SmartDashboard.getNumber("Goal RPM Outtake", 0);
        if (intakeShooter.getOuttakeRPM() >= goalRPM) {
            intakeShooter.setMaxIntake(1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeShooter.stopOuttake();
        intakeShooter.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return !intakeShooter.intakeDetectsNote()
                && !intakeShooter.outtakeDetectsNote();
    }
}
