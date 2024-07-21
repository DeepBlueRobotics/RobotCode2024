package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Limelightc.*;

import org.carlmontrobotics.subsystems.IntakeShooter;
import org.carlmontrobotics.subsystems.IntakeShooter.*;
import org.carlmontrobotics.subsystems.LimelightHelpers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class PassToOuttake extends Command {
    private final IntakeShooter intakeShooter;


    public PassToOuttake(IntakeShooter intakeShooter) {
        addRequirements(this.intakeShooter = intakeShooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (intakeShooter.getOuttakeRPM() >= 4000
                && intakeShooter.getOuttakeRPM() <= 4250) {
            intakeShooter.motorSetIntake(1);
        }

    }


    @Override
    public void end(boolean interrupted) {
        intakeShooter.resetCurrentLimit();
        intakeShooter.stopIntake();

    }

    @Override
    public boolean isFinished() {
        return (!intakeShooter.intakeDetectsNote()
                && !intakeShooter.outtakeDetectsNote());
        // || !LimelightHelpers.getTV(SHOOTER_LL_NAME);
    }
}
