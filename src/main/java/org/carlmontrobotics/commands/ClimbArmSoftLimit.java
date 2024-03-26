package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.Arm;

import static org.carlmontrobotics.Constants.Armc.*;

import org.carlmontrobotics.Constants.Armc.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbArmSoftLimit extends Command {
    //TODO: Debug this entire thing because it is 1 am and no way i did this correctly
    private Arm arm;
    public ClimbArmSoftLimit(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
        SmartDashboard.putNumber("climber volts", 0);
    }
    @Override
    public void initialize() {
        arm.setLimitsForClimbOn();
        arm.setBooleanDrive(false);
    }
    @Override
    public void execute() {
        arm.driveArm(SmartDashboard.getNumber("climber volts", 0));
    }
    @Override
    public void end(boolean interrupted) {
        arm.stopArm(); //OR maintain small voltage to keep arm in place?
        arm.resetSoftLimit();
        arm.setArmTarget(arm.getArmPos());
        arm.setBooleanDrive(true);
    }
    @Override
    public boolean isFinished() {
        //TODO: Figure out the actual climb position
        return Math.abs(arm.getArmPos() - CLIMBER_DOWN_ANGLE_RAD) < Units.degreesToRadians(5);
    }
}
