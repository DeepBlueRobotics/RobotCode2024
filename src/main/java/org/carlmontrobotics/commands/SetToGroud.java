package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Armc.GROUND_INTAKE_POS;

import org.carlmontrobotics.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.Command;

public class SetToGroud extends Command {
    private Arm arm;
    public SetToGroud(Arm arm) {
        this.arm = arm;
    }
    @Override
    public void initialize() {
        arm.setArmTarget(0);
    }
    @Override
    public void execute() {
        if(arm.getArmPos() <=0.02) {
            arm.stopArm();
        }
    }
    @Override
    public boolean isFinished() {
        return arm.getArmPos() <= GROUND_INTAKE_POS+0.15;
    }
}
