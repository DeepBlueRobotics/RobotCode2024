package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.IntakeShoot.*;
import static org.carlmontrobotics.Constants.IntakeShoot.OUTAKE_RPM;

import org.carlmontrobotics.subsystems.IntakeShooter;
import org.carlmontrobotics.Constants;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;

public class SpeakerShootRPM extends Command {
    //ramp the shooter up to a specific rpm
    private final IntakeShooter shooter;
    private double outakeRPM;
    public SpeakerShootRPM(IntakeShooter shooter, double rpm) {
        this.shooter = shooter;
        this.outakeRPM = rpm;
    }

    @Override
    public void initialize() {
      shooter.setRPMOutake(OUTAKE_RPM);
    }
    @Override
    public boolean isFinished() {
      return true;
    }
}