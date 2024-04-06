package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Armc.SMART_CURRENT_LIMIT_TIMEOUT;
import static org.carlmontrobotics.Constants.Effectorc.*;
import org.carlmontrobotics.subsystems.Arm;
import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwitchRPMShootNEO extends Command {
    private IntakeShooter intakeShooter;
    private double rpmAmount;
   
    
    public SwitchRPMShootNEO(IntakeShooter intakeShooter) {
        this.intakeShooter = intakeShooter;
        addRequirements(intakeShooter);
    }
    @Override
    public void initialize() {
        intakeShooter.setMaxOutake(1);
        
    }
    @Override
    public void execute() {
        rpmAmount = RPM_SELECTOR[Arm.getSelector()];
        if(intakeShooter.getOutakeRPM() >= rpmAmount) {
        intakeShooter.setMaxIntake(1);
       
        }
    }
    @Override
    public void end(boolean interrupted) {
        intakeShooter.stopIntake();
        intakeShooter.stopOutake();
        intakeShooter.resetCurrentLimit();
      
        
    }
    @Override
    public boolean isFinished() {
        return (!intakeShooter.intakeDetectsNote() && !intakeShooter.outakeDetectsNote());
    }
}
