package org.carlmontrobotics.commands;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.Arm;
import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwitchRPMShoot extends Command {
    private IntakeShooter intakeShooter;
    private double rpmAmount;
    private Timer timer = new Timer();
    
    public SwitchRPMShoot(IntakeShooter intakeShooter) {
        this.intakeShooter = intakeShooter;
        rpmAmount = Constants.Effectorc.RPM_SELECTOR[Arm.getSelector()];
    }
    @Override
    public void initialize() {
        intakeShooter.setMaxOutake();
        timer.reset();
        SmartDashboard.putNumber("RPM amount for current angle", rpmAmount);
    }
    @Override
    public void execute() {
        if(intakeShooter.getOutakeRPM() >= rpmAmount) {
        intakeShooter.setMaxIntake(1);
        timer.start();
        }
    }
    @Override
    public void end(boolean interrupted) {
        intakeShooter.stopIntake();
        intakeShooter.stopOutake();
        intakeShooter.resetCurrentLimit();
        timer.stop();
    }
    @Override
    public boolean isFinished() {
        return (!intakeShooter.intakeDetectsNote() && !intakeShooter.outakeDetectsNote()) || timer.get()>0.9;
    }
}
