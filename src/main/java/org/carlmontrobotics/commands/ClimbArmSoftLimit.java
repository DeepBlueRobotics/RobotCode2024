package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Armc.GROUND_INTAKE_POS;

import org.carlmontrobotics.subsystems.Arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbArmSoftLimit extends Command {
    // TODO: Debug this entire thing because it is 1 am and no way i did this
    // correctly
    private Arm arm;
    private Timer timer = new Timer();
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
        arm.driveArm(-12);
        timer.start();
        if(timer.get() > 22) {
            
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        arm.stopArm(); // OR maintain small voltage to keep arm in place?
        arm.resetSoftLimit();
        // arm.setArmTarget(arm.getArmPos());
        arm.setBooleanDrive(true);
        arm.resetGoal();
        timer.stop();
        timer.reset();
        arm.setPIDOff(true);
    }

    @Override
    public boolean isFinished() {
        // TODO: Figure out the actual climb position
        return Math.abs(arm.getArmPos() - GROUND_INTAKE_POS) < Units.degreesToRadians(2) || timer.get() > 25;
    }
}
