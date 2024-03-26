package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.Arm;

import static org.carlmontrobotics.Constants.Armc.*;

import org.carlmontrobotics.Constants.Armc.*;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbArmSoftLimit extends Command {
    //TODO: Debug this entire thing because it is 1 am and no way i did this correctly
    private Arm arm;
    public ClimbArmSoftLimit(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }
    @Override
    public void initialize() {
        arm.setLimitsForClimbOn();
        arm.setArmTarget(CLIMBER_UP_ANGLE_RAD);
        arm.setBooleanDrive(true);
    }
    @Override
    public void execute() {
    arm.driveArmMax(-1);
    arm.setSoftLimit( (float) SOFT_LIMIT_LOCATION_IN_RADIANS); //TODO: Find out what limit actually is defined as
    }
    @Override
    public void end(boolean interrupted) {
        arm.stopArm(); //OR maintain small voltage to keep arm in place?
        arm.resetSoftLimit();
        arm.setBooleanDrive(false);
    }
    @Override
    public boolean isFinished() {
        return arm.getArmPos() <= CLIMBER_DOWN_ANGLE_RAD + 0.02 || arm.getArmPos() >= CLIMBER_DOWN_ANGLE_RAD - 0.05; //TODO: Figure out the actual climb position
    }
}
