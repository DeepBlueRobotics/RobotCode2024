package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Armc.*;

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
        return arm.getArmPos() - (CLIMB_FINISH_POS) < Units.degreesToRadians(0.1) || timer.get() >= 5;
    }
}