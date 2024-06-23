package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Armc.PODIUM_ANGLE_RAD;
import static org.carlmontrobotics.Constants.Armc.SMART_CURRENT_LIMIT_TIMEOUT;
import static org.carlmontrobotics.Constants.Armc.SPEAKER_ANGLE_RAD;
import static org.carlmontrobotics.Constants.Armc.SUBWOOFER_ANGLE_RAD;
import static org.carlmontrobotics.Constants.Effectorc.*;
import org.carlmontrobotics.subsystems.Arm;
import org.carlmontrobotics.subsystems.IntakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwitchRPMShootNEO extends Command {
    private IntakeShooter intakeShooter;
    private Arm arm;
    private double currentPos;
    private double rpmAmount;
    private Timer timer = new Timer();
   
    
    public SwitchRPMShootNEO(IntakeShooter intakeShooter, Arm arm) {
        this.intakeShooter = intakeShooter;
        this.arm = arm;
        addRequirements(intakeShooter, arm);
    }
    @Override
    public void initialize() {
        intakeShooter.setMaxOuttake(1);
        currentPos = arm.getArmPos();
        if (currentPos >= PODIUM_ANGLE_RAD + 0.1
                && currentPos <= PODIUM_ANGLE_RAD - 0.1) {
            rpmAmount = PODIUM_RPM;
        } else if (currentPos >= SPEAKER_ANGLE_RAD - 0.1) {
            rpmAmount = SPEAKER_RPM;
        } else {
            rpmAmount = SUBWOOFER_RPM;
        }

    }

    @Override
    public void execute() {

        if (intakeShooter.getOuttakeRPM() >= rpmAmount) {
        intakeShooter.setMaxIntake(1);
        timer.start();
        }
    }
    @Override
    public void end(boolean interrupted) {
        intakeShooter.stopIntake();
        intakeShooter.stopOuttake();
        intakeShooter.resetCurrentLimit();
        timer.stop();
        
    }
    @Override
    public boolean isFinished() {
        return (!intakeShooter.intakeDetectsNote() && !intakeShooter.outtakeDetectsNote()) || timer.get() > 10;
    }
}
