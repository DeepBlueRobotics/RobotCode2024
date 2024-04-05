package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Armc.SMART_CURRENT_LIMIT_TIMEOUT;
import static org.carlmontrobotics.Constants.Effectorc.*;
import org.carlmontrobotics.subsystems.Arm;
import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwitchRPMShoot extends Command {
    private IntakeShooter intakeShooter;
    private double rpmAmount;
    private Timer timer = new Timer();
    private static Timer timer2 = new Timer();
    
    public SwitchRPMShoot(IntakeShooter intakeShooter) {
        this.intakeShooter = intakeShooter;
        timer2.stop();
        timer2.reset();
        addRequirements(intakeShooter);
    }
    @Override
    public void initialize() {
        intakeShooter.setMaxOutake(1);
        timer.reset();
        
    }
    @Override
    public void execute() {
        rpmAmount = RPM_SELECTOR[Arm.getSelector()];
        if(intakeShooter.getOutakeRPM() >= rpmAmount) {
        intakeShooter.setMaxIntake(1);
        timer.start();
        timer.reset();
        }
    }
    @Override
    public void end(boolean interrupted) {
        intakeShooter.stopIntake();
        intakeShooter.stopOutake();
        intakeShooter.resetCurrentLimit();
        timer.stop();
        timer2.start();
    }
    @Override
    public boolean isFinished() {
        return (!intakeShooter.intakeDetectsNote() && !intakeShooter.outakeDetectsNote()) || timer.get()>SMART_CURRENT_LIMIT_TIMEOUT;
    }
}
