package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.IntakeShoot.*;
import static org.carlmontrobotics.Constants.Led.*;
import static org.carlmontrobotics.Constants.Arm.*;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.Led;
import org.w3c.dom.events.MouseEvent;
import org.carlmontrobotics.subsystems.IntakeShooter;
import org.carlmontrobotics.subsystems.Arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
public class MoveToPos extends Command {
    //intake until sees game peice or 4sec has passed
    private final Timer timer = new Timer();
   
    private final Arm arm;
    private double goal;
    public MoveToPos(Arm arm, double goal) {
        
        this.arm = arm;
        this.goal = goal;
        addRequirements(arm);

    }    
    
    @Override
    public void initialize() {
      arm.setArmTarget(goal);
      
      timer.reset();
      timer.start();
    }
      

  // Called every time the scheduler runs while the command is scheduled.
  @Override
    public void execute() {
      //Intake Led
      
      
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    //resets to defaultColor
    

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.armAtSetpoint();
  }
}
