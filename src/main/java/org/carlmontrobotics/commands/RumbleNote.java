package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RumbleNote extends Command{
    Timer timer = new Timer();
    IntakeShooter intake;
    GenericHID controller;
    public RumbleNote(IntakeShooter intake, GenericHID controller) {
        this.intake = intake;
        this.controller = controller;
        addRequirements(intake);
        SmartDashboard.putNumber("rumble amount",0);
    }
    @Override
    public void initialize() {
        timer.reset();
        controller.setRumble(RumbleType.kBothRumble, 0);
        SmartDashboard.putNumber("rumble amount",0);
    }
    @Override
    public void execute() {
        
        if(intake.intakeDetectsNote() && !intake.getRumblyTumbly()) {
            timer.start();
            SmartDashboard.putNumber("rumble amount",1);
            controller.setRumble(RumbleType.kBothRumble, 1);
        }
        if(timer.get()>3) {
            intake.setRumblyTumbly(true);
        } else {
            return;
        }
    }
    @Override
    public void end(boolean interrupted) {
        
        controller.setRumble(RumbleType.kBothRumble, 0);
        SmartDashboard.putNumber("rumble amount",0);
        timer.stop();
    }
    @Override
    public boolean isFinished() {
        
        return timer.get() > 1.5;
    }
}
