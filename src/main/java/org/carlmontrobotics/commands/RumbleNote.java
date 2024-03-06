package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class RumbleNote extends Command{
    IntakeShooter intake;
    GenericHID controller;
    public RumbleNote(IntakeShooter intakle, GenericHID controller) {
        this.intake = intake;
        this.controller = controller;
    }
    @Override
    public void initialize() {
        controller.setRumble(RumbleType.kBothRumble, 0);
    }
    @Override
    public void execute() {
        if(intake.intakeDetectsNote()) {
            controller.setRumble(RumbleType.kBothRumble, 0.5);
        }
    }
    @Override
    public void end(boolean interrupted) {
        controller.setRumble(RumbleType.kBothRumble, 0);
    }
    @Override
    public boolean isFinished() {
        return !intake.intakeDetectsNote();
    }
}
